// Landing Target 3D - UDP MAVLink Server
// Connects to Mission Planner via UDP and serves visualization via WebSocket

const dgram = require('dgram');
const http = require('http');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');

// Configuration
const CONFIG = {
    MAVLINK_HOST: process.argv[2] || '127.0.0.1',
    MAVLINK_PORT: parseInt(process.argv[3]) || 14570,
    HTTP_PORT: parseInt(process.argv[4]) || 8080,
    LOCAL_PORT: 14561  // Local port to bind for receiving
};

console.log(`Landing Target 3D Server`);
console.log(`========================`);
console.log(`MAVLink Target: ${CONFIG.MAVLINK_HOST}:${CONFIG.MAVLINK_PORT}`);
console.log(`HTTP Server: http://localhost:${CONFIG.HTTP_PORT}`);
console.log(`========================`);

// MAVLink parser state
const mavlinkParser = {
    buffer: Buffer.alloc(0),
    MAVLINK_STX_V1: 0xFE,
    MAVLINK_STX_V2: 0xFD,

    // MAVLink message IDs we care about
    MSG_ATTITUDE: 30,
    MSG_LANDING_TARGET: 149,

    parse(data) {
        this.buffer = Buffer.concat([this.buffer, data]);
        const messages = [];

        while (this.buffer.length > 0) {
            // Find start byte
            let startIdx = -1;
            for (let i = 0; i < this.buffer.length; i++) {
                if (this.buffer[i] === this.MAVLINK_STX_V1 || this.buffer[i] === this.MAVLINK_STX_V2) {
                    startIdx = i;
                    break;
                }
            }

            if (startIdx === -1) {
                this.buffer = Buffer.alloc(0);
                break;
            }

            if (startIdx > 0) {
                this.buffer = this.buffer.slice(startIdx);
            }

            const stx = this.buffer[0];

            if (stx === this.MAVLINK_STX_V1) {
                // MAVLink v1
                if (this.buffer.length < 8) break;

                const len = this.buffer[1];
                const msgLen = 8 + len;

                if (this.buffer.length < msgLen) break;

                const msgId = this.buffer[5];
                const payload = this.buffer.slice(6, 6 + len);

                const msg = this.parseMessage(msgId, payload);
                if (msg) messages.push(msg);

                this.buffer = this.buffer.slice(msgLen);
            } else if (stx === this.MAVLINK_STX_V2) {
                // MAVLink v2
                if (this.buffer.length < 12) break;

                const len = this.buffer[1];
                const incompatFlags = this.buffer[2];
                const msgLen = 12 + len + (incompatFlags & 0x01 ? 13 : 0);

                if (this.buffer.length < msgLen) break;

                const msgId = this.buffer[7] | (this.buffer[8] << 8) | (this.buffer[9] << 16);
                const payload = this.buffer.slice(10, 10 + len);

                const msg = this.parseMessage(msgId, payload);
                if (msg) messages.push(msg);

                this.buffer = this.buffer.slice(msgLen);
            } else {
                this.buffer = this.buffer.slice(1);
            }
        }

        // Prevent buffer from growing too large
        if (this.buffer.length > 10000) {
            this.buffer = this.buffer.slice(-1000);
        }

        return messages;
    },

    parseMessage(msgId, payload) {
        if (msgId === this.MSG_ATTITUDE) {
            return this.parseAttitude(payload);
        } else if (msgId === this.MSG_LANDING_TARGET) {
            return this.parseLandingTarget(payload);
        }
        return null;
    },

    parseAttitude(payload) {
        if (payload.length < 28) return null;

        return {
            _id: 30,
            _name: 'ATTITUDE',
            time_boot_ms: payload.readUInt32LE(0),
            roll: payload.readFloatLE(4),
            pitch: payload.readFloatLE(8),
            yaw: payload.readFloatLE(12),
            rollspeed: payload.readFloatLE(16),
            pitchspeed: payload.readFloatLE(20),
            yawspeed: payload.readFloatLE(24)
        };
    },

    parseLandingTarget(payload) {
        if (payload.length < 30) return null;

        const msg = {
            _id: 149,
            _name: 'LANDING_TARGET',
            time_usec: Number(payload.readBigUInt64LE(0)),
            target_num: payload.readUInt8(8),
            frame: payload.readUInt8(9),
            angle_x: payload.readFloatLE(10),
            angle_y: payload.readFloatLE(14),
            distance: payload.readFloatLE(18),
            size_x: payload.readFloatLE(22),
            size_y: payload.readFloatLE(26)
        };

        // Extended fields (if available)
        if (payload.length >= 42) {
            msg.x = payload.readFloatLE(30);
            msg.y = payload.readFloatLE(34);
            msg.z = payload.readFloatLE(38);
        }

        if (payload.length >= 58) {
            msg.q = [
                payload.readFloatLE(42),
                payload.readFloatLE(46),
                payload.readFloatLE(50),
                payload.readFloatLE(54)
            ];
        }

        if (payload.length >= 59) {
            msg.type = payload.readUInt8(58);
        }

        if (payload.length >= 60) {
            msg.position_valid = payload.readUInt8(59);
        }

        return msg;
    }
};

// UDP Client
const udpClient = dgram.createSocket('udp4');
let mavlinkConnected = false;

udpClient.on('message', (data, rinfo) => {
    if (!mavlinkConnected) {
        console.log(`Receiving MAVLink from ${rinfo.address}:${rinfo.port}`);
        mavlinkConnected = true;
    }

    const messages = mavlinkParser.parse(data);

    for (const msg of messages) {
        // Broadcast to all WebSocket clients
        wss.clients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify(msg));
            }
        });
    }
});

udpClient.on('error', (err) => {
    console.error(`UDP error: ${err.message}`);
});

// Bind to local port and send initial packet to establish connection
udpClient.bind(CONFIG.LOCAL_PORT, () => {
    console.log(`UDP bound to local port ${CONFIG.LOCAL_PORT}`);

    // Send a heartbeat to initiate connection (Mission Planner needs this)
    const heartbeat = Buffer.from([
        0xFD, // STX v2
        0x09, // Payload length
        0x00, // Incompat flags
        0x00, // Compat flags
        0x00, // Sequence
        0xFF, // System ID
        0x00, // Component ID
        0x00, 0x00, 0x00, // Message ID (HEARTBEAT = 0)
        // Payload
        0x00, 0x00, 0x00, 0x00, // custom_mode
        0x06, // type (GCS)
        0x08, // autopilot (INVALID)
        0x00, // base_mode
        0x00, // system_status
        0x03, // mavlink_version
        // CRC (placeholder - would need proper CRC calculation)
        0x00, 0x00
    ]);

    setInterval(() => {
        udpClient.send(heartbeat, CONFIG.MAVLINK_PORT, CONFIG.MAVLINK_HOST, (err) => {
            if (err && !mavlinkConnected) {
                console.log(`Waiting for MAVLink connection to ${CONFIG.MAVLINK_HOST}:${CONFIG.MAVLINK_PORT}...`);
            }
        });
    }, 1000);
});

// HTTP Server
const httpServer = http.createServer((req, res) => {
    let filePath = req.url === '/' ? '/index.html' : req.url;
    filePath = path.join(__dirname, filePath);

    const extname = path.extname(filePath);
    const contentTypes = {
        '.html': 'text/html',
        '.js': 'text/javascript',
        '.css': 'text/css',
        '.json': 'application/json'
    };

    fs.readFile(filePath, (err, content) => {
        if (err) {
            res.writeHead(404);
            res.end('Not Found');
        } else {
            res.writeHead(200, { 'Content-Type': contentTypes[extname] || 'text/plain' });
            res.end(content);
        }
    });
});

// WebSocket Server
const wss = new WebSocket.Server({ server: httpServer });

wss.on('connection', (ws) => {
    console.log('WebSocket client connected');

    ws.on('close', () => {
        console.log('WebSocket client disconnected');
    });
});

httpServer.listen(CONFIG.HTTP_PORT, () => {
    console.log(`HTTP server running at http://localhost:${CONFIG.HTTP_PORT}`);
});
