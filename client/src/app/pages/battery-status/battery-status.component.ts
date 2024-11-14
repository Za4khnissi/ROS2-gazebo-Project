import { Component, Input, OnInit, OnDestroy } from '@angular/core';
import { CommonModule } from '@angular/common';
import { WebSocketService } from '@app/services/web-socket.service';
import { Subscription } from 'rxjs'; 

@Component({
  selector: 'app-battery-status',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './battery-status.component.html',
  styleUrl: './battery-status.component.css'
})
export class BatteryStatusComponent implements OnInit, OnDestroy {
  @Input() robotId!: number; 
  @Input() mode!: 'simulation' | 'physical'; 
  batteryLevel: number = 100;
  
  private batterySubscription!: Subscription;

  constructor(private socketBattery: WebSocketService) {}

  ngOnInit(): void {
    console.log('Robot ID:', this.robotId);

  
    this.batterySubscription = this.socketBattery.listenToBatteryStatus(this.robotId, this.mode).subscribe(
      (level: number) => {
        this.batteryLevel = Math.floor(level);
        console.log(`Battery level updated: ${this.batteryLevel}%`);
      },
      (error) => {
        console.error('Error receiving battery status:', error);
      }
    );
  }

  getBatteryColor(): string {
    if (this.batteryLevel > 50) {
      return 'green';
    } else if (this.batteryLevel > 20) {
      return 'yellow';
    } else {
      return 'red';
    }
  }
  

  ngOnDestroy(): void {
    if (this.batterySubscription) {
      this.batterySubscription.unsubscribe();
    }
  }
}
