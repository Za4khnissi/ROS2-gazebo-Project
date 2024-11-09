import { Schema, Document } from 'mongoose';

export interface MissionModel extends Document {
  dateDebut: Date;
  dateFin: Date;
  duration: number;
  robots: string[];
  isPhysical: boolean;
  totalDistance: number;
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
}, { timestamps: true }); 

export const Mission = { name: 'Mission', schema: MissionSchema };
