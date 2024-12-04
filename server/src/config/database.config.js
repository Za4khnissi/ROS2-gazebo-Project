"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var mongoose_1 = require("mongoose");
var mission_model_1 = require("../mission/mission.model");
// Définissez le modèle Mission en utilisant le schéma importé
var Mission = mongoose_1.default.model('Mission', mission_model_1.MissionSchema);
function seedMissions() {
    return __awaiter(this, void 0, void 0, function () {
        var missions, error_1;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    _a.trys.push([0, 4, 5, 7]);
                    return [4 /*yield*/, mongoose_1.default.connect('mongodb+srv://axellestevialetieutchemeni:projet3@cluster0.urbxk.mongodb.net/<database_name>', {})];
                case 1:
                    _a.sent();
                    console.log('Connected to MongoDB');
                    // Supprimez toutes les missions existantes
                    return [4 /*yield*/, Mission.deleteMany({})];
                case 2:
                    // Supprimez toutes les missions existantes
                    _a.sent();
                    console.log('All missions deleted successfully');
                    missions = [
                        {
                            dateDebut: new Date('2024-11-06T18:15:00Z'),
                            dateFin: new Date('2024-11-06T20:15:00Z'),
                            duration: 8000, // durée en secondes (2 heures)
                            robots: ['limo_105_3'],
                            isPhysical: true,
                            totalDistance: 150, // distance en unités arbitraires
                            logs: [
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Configuring backup',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Creating behavior plugin drive_on_heading of type nav2_behaviors/DriveOnHeading',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Configuring drive_on_heading',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Creating behavior plugin wait of type nav2_behaviors/Wait',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Configuring wait',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Configuring bt_navigator',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                                {
                                    robotId: 'limo_105_3',
                                    level: 20,
                                    message: 'Configuring',
                                    timestamp: new Date('2024-11-06T20:15:00Z'),
                                },
                            ],
                        },
                        {
                            dateDebut: new Date('2024-11-05T10:00:00Z'),
                            dateFin: new Date('2024-11-05T12:30:00Z'),
                            duration: 9000, // durée en secondes (2,5 heures)
                            robots: ['limo_105_4'],
                            isPhysical: false,
                            totalDistance: 200, // distance en unités arbitraires
                            logs: [
                                {
                                    robotId: 'limo_105_4',
                                    level: 15,
                                    message: 'Robot initialized',
                                    timestamp: new Date('2024-11-05T10:00:00Z'),
                                },
                                {
                                    robotId: 'limo_105_4',
                                    level: 10,
                                    message: 'Starting navigation',
                                    timestamp: new Date('2024-11-05T10:05:00Z'),
                                },
                                {
                                    robotId: 'limo_105_4',
                                    level: 20,
                                    message: 'Avoiding obstacle',
                                    timestamp: new Date('2024-11-05T11:30:00Z'),
                                },
                                {
                                    robotId: 'limo_105_4',
                                    level: 30,
                                    message: 'Mission complete',
                                    timestamp: new Date('2024-11-05T12:30:00Z'),
                                },
                            ],
                        },
                    ];
                    // Ajoutez les nouvelles missions
                    return [4 /*yield*/, Mission.insertMany(missions)];
                case 3:
                    // Ajoutez les nouvelles missions
                    _a.sent();
                    console.log('Missions added successfully');
                    return [3 /*break*/, 7];
                case 4:
                    error_1 = _a.sent();
                    console.error('Error seeding missions:', error_1);
                    return [3 /*break*/, 7];
                case 5: 
                // Déconnectez-vous de MongoDB
                return [4 /*yield*/, mongoose_1.default.disconnect()];
                case 6:
                    // Déconnectez-vous de MongoDB
                    _a.sent();
                    console.log('Disconnected from MongoDB');
                    return [7 /*endfinally*/];
                case 7: return [2 /*return*/];
            }
        });
    });
}
// Exécutez la fonction de peuplement
seedMissions();
