import { Component } from '@angular/core';
import { OnInit } from '@angular/core';
import { ActivatedRoute } from '@angular/router';
import { CommonModule } from '@angular/common';
import { BatteryStatusComponent } from '../battery-status/battery-status.component';
import { MapComponent } from '../../components/map/map.component';

@Component({
  selector: 'app-mission',
  standalone: true,
  imports: [BatteryStatusComponent,CommonModule, MapComponent],
  templateUrl: './mission.component.html',
  styleUrl: './mission.component.css'
})
export class MissionComponent implements OnInit {
  robotId: number= 0;
  mode: 'simulation' | 'physical' = 'simulation';
  constructor(private route: ActivatedRoute) {}
  ngOnInit(): void { 
    this.robotId = Number(this.route.snapshot.paramMap.get('robotId'));
    const mode = this.route.snapshot.paramMap.get('mode') as 'simulation' | 'physical';
    this.loadData();
  }

  loadData(){
    console.log(this.robotId);
    console.log(`Robot ID: ${this.robotId}, Mode: ${this.mode}`);
  }


}
