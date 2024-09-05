import { Routes } from '@angular/router';

const routes: Routes = [
    { path: 'home', component: HomeComponent },
    { path: 'robot/physical', component: PhysicalRobotComponent },
    { path: 'robot/simulation', component: SimulationComponent },
    { path: '', redirectTo: '/home', pathMatch: 'full' },
  ];
