import mongoose from 'mongoose';
import { MissionSchema } from '../mission/mission.model';

const Mission = mongoose.model('Mission', MissionSchema);

async function seedMissions() {
  try {
    await mongoose.connect('mongodb+srv://axellestevialetieutchemeni:projet3@cluster0.urbxk.mongodb.net/<database_name>', {
     
    });
    console.log('Connected to MongoDB');

    await Mission.deleteMany({});
    console.log('All missions deleted successfully');

    const missions = [
      {
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 1208, 
        robots: ['robot5'],
        isPhysical: true,
        totalDistance: 150,
      },
      {
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 198,
        robots: ['robot2', 'robot3'],
        isPhysical: false,
        totalDistance: 1000,
      },
      {
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 345,
        robots: ['robot9'],
        isPhysical: true,
        totalDistance: 300,
      },
    ];

    await Mission.insertMany(missions);
    console.log('Missions added successfully');
  } catch (error) {
    console.error('Error seeding missions:', error);
  } finally {
    mongoose.disconnect();
    console.log('Disconnected from MongoDB');
  }
}

seedMissions();
