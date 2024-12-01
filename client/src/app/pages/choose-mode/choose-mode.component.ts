import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { CommonModule } from '@angular/common';

@Component({
  selector: 'app-choose-mode',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './choose-mode.component.html',
  styleUrl: './choose-mode.component.css'
})
export class ChooseModeComponent {
  constructor(private router: Router) {}
  isHelpVisible = false;

  goToPhysicalRobot() {
    this.router.navigate(['/robot/physical'])
  }

  goToSimulation() {
    this.router.navigate(['/robot/simulation']);
  }

  goToHistory()
  {
    this.router.navigate(['/robot/history']);
  }

  toggleHelp() {
    this.isHelpVisible = !this.isHelpVisible;
  }


}
