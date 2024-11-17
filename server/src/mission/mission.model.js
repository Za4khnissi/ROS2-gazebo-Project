"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.Mission = exports.MissionSchema = void 0;
var mongoose_1 = require("mongoose");
exports.MissionSchema = new mongoose_1.Schema({
    dateDebut: {
        type: Date,
        required: true,
    },
    dateFin: {
        type: Date,
        required: false,
    },
    duration: {
        type: Number,
        required: false,
    },
    robots: {
        type: [String],
        required: true,
    },
    isPhysical: {
        type: Boolean,
        required: true,
    },
    totalDistance: {
        type: Number,
        required: false,
    },
    mapData: {
        type: mongoose_1.Schema.Types.Mixed
    },
    logs: [
        {
            robotId: { type: String, required: false },
            event: { type: String, required: false },
            level: { type: String, required: false },
            message: { type: String, required: true },
            timestamp: { type: Date, required: true, default: Date.now },
        },
    ],
}, { timestamps: true });
exports.Mission = { name: 'Mission', schema: exports.MissionSchema };
