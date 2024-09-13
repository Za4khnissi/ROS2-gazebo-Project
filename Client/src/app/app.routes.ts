import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { HomeComponent } from '@app/pages/home/home.component';
import { PhysicalRobotComponent } from '@app/pages/robot/physical/physical.component';
import { SimulationComponent } from '@app/pages/robot/simulation/simulation.component';

export const routes: Routes = [
  { path: '', component: HomeComponent },
  { path: 'home', component: HomeComponent },
  { path: 'robot/physical', component: PhysicalRobotComponent },
  { path: 'robot/simulation', component: SimulationComponent },
  { path: '**', redirectTo: '/home' }
  ];

  @NgModule({
    imports: [RouterModule.forRoot(routes)],
    exports: [RouterModule]
  })

  export class AppRoutingModule {}

  