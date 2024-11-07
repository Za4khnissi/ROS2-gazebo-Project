import { NgModule, NO_ERRORS_SCHEMA } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { provideHttpClient } from '@angular/common/http';
import { AppComponent } from './app.component';
import { AppRoutingModule } from './app.routes';
import { HomeComponent } from '@app/pages/home/home.component';
import { PhysicalRobotComponent } from '@app/pages/physical/physical.component';
import { SimulationComponent } from '@app/pages/simulation/simulation.component';
import { BatteryStatusComponent } from '@app/pages/battery-status/battery-status.component'; 
import { ChooseModeComponent } from './pages/choose-mode/choose-mode.component';
import { CommonModule } from '@angular/common';
import { HistoryComponent } from './pages/history/history.component';
import {MissionComponent} from '@app/pages/mission/mission.component';
import { FormsModule } from '@angular/forms';


@NgModule({
  declarations: [
    AppComponent,
    HomeComponent,
    BatteryStatusComponent,
    ChooseModeComponent,
    PhysicalRobotComponent,
    HistoryComponent,
    MissionComponent,
    SimulationComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    CommonModule,
    FormsModule
  ],
  schemas: [NO_ERRORS_SCHEMA],
  providers: [provideHttpClient()],
  bootstrap: [AppComponent]
})
export class AppModule {}
