import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { HomeComponent } from '@app/pages/home/home.component';
import { PhysicalRobotComponent } from '@app/pages/physical/physical.component';
import { SimulationComponent } from '@app/pages/simulation/simulation.component';
import { BatteryStatusComponent } from './pages/battery-status/battery-status.component';
import { ChooseModeComponent } from './pages/choose-mode/choose-mode.component';
import { HistoryComponent } from './pages/history/history.component';
import { MissionComponent } from './pages/mission/mission.component';


export const routes: Routes = [
  { path: '', component: HomeComponent },
  { path: 'home', component: HomeComponent },
  { path: 'robot/physical', component: PhysicalRobotComponent },
  { path: 'robot/simulation', component: SimulationComponent },
  { path: 'robot/history', component: HistoryComponent },
  { path: 'battery-status', component: BatteryStatusComponent },
  { path: 'choose-mode', component: ChooseModeComponent },
  {path: 'mission/:robotId/:mode', component: MissionComponent},
  { path: '**', redirectTo: '/home' }
  ];

  @NgModule({
    imports: [RouterModule.forRoot(routes)],
    exports: [RouterModule]
  })

  export class AppRoutingModule {}

  