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

    // Missions à insérer
    const missions = [
      {
        dateDebut: new Date(),
        dateFin: new Date(),
        duration: 1208,
        robots: ['robot5'],
        isPhysical: true,
        totalDistance: 150,
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
