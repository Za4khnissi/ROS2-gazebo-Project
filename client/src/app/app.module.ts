import { NgModule, NO_ERRORS_SCHEMA } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { provideHttpClient } from '@angular/common/http';
import { AppComponent } from './app.component';
import { AppRoutingModule } from './app.routes';
import { HomeComponent } from '@app/pages/home/home.component';
import { PhysicalRobotComponent } from '@app/pages/physical/physical.component';
import { SimulationComponent } from '@app/pages/simulation/simulation.component';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';


@NgModule({
  declarations: [
    AppComponent,
    HomeComponent,
    PhysicalRobotComponent,
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
