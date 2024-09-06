import { Component } from '@angular/core';
import { RobotService } from '@app/services/phy-robot.service';

@Component({
  selector: 'app-physical-robot',
  templateUrl: './physical.component.html',
  styleUrls: ['./physical.component.css']
})
export class PhysicalRobotComponent {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';

  constructor(private robotService: RobotService) {}

  identifyRobot(robotId: number) {
    this.robotService.identifyRobot(robotId).subscribe({
      next: (response) => {
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error identifying robot:', error);
      }
    });
  }

  startMission(robotId: number) {
    this.robotService.startMission(robotId).subscribe({
      next: (response) => {
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error starting mission:', error);
      }
    });
  }

  stopMission(robotId: number) {
    this.robotService.stopMission(robotId).subscribe({
      next: (response) => {
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error stopping mission:', error);
      }
    });
  }
}
