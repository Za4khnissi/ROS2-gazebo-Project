import { Schema, Document } from 'mongoose';

export interface MissionLog {
  robotId: string;
  event: string;
  level: string;
  message: string;
  timestamp: Date;
}

export interface MissionModel extends Document {
  dateDebut: Date;
  dateFin: Date;
  duration: number;
  robots: string[];
  isPhysical: boolean;
  totalDistance: number;
  mapData: any;
  logs: MissionLog[]; 
}

export const MissionSchema = new Schema<MissionModel>({
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
    type: Schema.Types.Mixed 
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

export const Mission = { name: 'Mission', schema: MissionSchema };
