import mongoose from 'mongoose';
import { MissionSchema } from '../mission/mission.model';

// Définissez le modèle Mission en utilisant le schéma importé
const Mission = mongoose.model('Mission', MissionSchema);

async function seedMissions() {
  try {
    await mongoose.connect('mongodb+srv://axellestevialetieutchemeni:projet3@cluster0.urbxk.mongodb.net/<database_name>', {
    });
    console.log('Connected to MongoDB');

    // Supprimez toutes les missions existantes
    await Mission.deleteMany({});
    console.log('All missions deleted successfully');

    // Exemples de missions avec des logs
    const missions = [
      {
        dateDebut: new Date('2024-11-06T18:15:00Z'),
        dateFin: new Date('2024-11-06T20:15:00Z'),
        duration: 7200, // durée en secondes (2 heures)
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
    await Mission.insertMany(missions);
    console.log('Missions added successfully');
  } catch (error) {
    console.error('Error seeding missions:', error);
  } finally {
    // Déconnectez-vous de MongoDB
    await mongoose.disconnect();
    console.log('Disconnected from MongoDB');
  }
}

// Exécutez la fonction de peuplement
seedMissions();
