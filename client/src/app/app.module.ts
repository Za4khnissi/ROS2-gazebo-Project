import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { provideHttpClient } from '@angular/common/http';
import { AppComponent } from './app.component';
import { AppRoutingModule } from './app.routes';
import { HomeComponent } from '@app/pages/home/home.component';
import { PhysicalRobotComponent } from '@app/pages/robot/physical/physical.component';
import { SimulationComponent } from '@app/pages/robot/simulation/simulation.component';


@NgModule({
  declarations: [
    AppComponent,
    HomeComponent,
    PhysicalRobotComponent,
    SimulationComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule
  ],
  providers: [provideHttpClient()],
  bootstrap: [AppComponent]
})
export class AppModule {}