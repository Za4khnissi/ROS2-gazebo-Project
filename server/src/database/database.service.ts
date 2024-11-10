import { Injectable } from '@nestjs/common';
import { Model } from 'mongoose';
import { InjectModel } from '@nestjs/mongoose';
import { Mission, MissionModel } from '../mission/mission.model';

@Injectable()
export class DatabaseService {
  constructor(
    @InjectModel(Mission.name) private missionModel: Model<MissionModel>, 
  ) {}

  
  async getAllMissions(): Promise<MissionModel[]> {
    const missions = await this.missionModel.find().exec();
    console.log('Fetched missions:', missions);  
    return missions;
  }

  
  async getMissionById(missionId: string): Promise<MissionModel> {
    return this.missionModel.findById(missionId).exec();
  }
  
  async createMission(missionDto: MissionModel): Promise<MissionModel> {
    const newMission = new this.missionModel(missionDto);
    return newMission.save();
  }

  async updateMission(missionId: string, missionDto: MissionModel): Promise<MissionModel> {
    return this.missionModel.findByIdAndUpdate(missionId, missionDto, { new: true }).exec();
  }

  async deleteMission(missionId: string): Promise<MissionModel> {
    return this.missionModel.findByIdAndDelete(missionId).exec();
  }
}
