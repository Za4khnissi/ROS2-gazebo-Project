import { Component } from '@angular/core';
import { Router } from '@angular/router';

@Component({
  selector: 'app-choose-mode',
  standalone: true,
  imports: [],
  templateUrl: './choose-mode.component.html',
  styleUrl: './choose-mode.component.css'
})
export class ChooseModeComponent {
  constructor(private router: Router) {}

  goToPhysicalRobot() {
    this.router.navigate(['/robot/physical'])
  }

  goToSimulation() {
    this.router.navigate(['/robot/simulation']);
  }

  goToHistory()
  {
    //this.router.navigate(['/robot/history']);
  }


}
