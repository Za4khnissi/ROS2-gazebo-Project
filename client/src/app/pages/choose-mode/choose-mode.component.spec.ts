import { ComponentFixture, TestBed } from '@angular/core/testing';
import { ChooseModeComponent } from './choose-mode.component';
import { Router } from '@angular/router';
import { CommonModule } from '@angular/common';
import { RouterTestingModule } from '@angular/router/testing';

describe('ChooseModeComponent', () => {
  let component: ChooseModeComponent;
  let fixture: ComponentFixture<ChooseModeComponent>;
  let router: Router;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [ChooseModeComponent, RouterTestingModule.withRoutes([]), CommonModule], // Utiliser 'imports' pour les composants standalone
    }).compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(ChooseModeComponent);
    component = fixture.componentInstance;
    router = TestBed.inject(Router);
    fixture.detectChanges();
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });

  it('should navigate to /robot/physical when goToPhysicalRobot is called', () => {
    const navigateSpy = spyOn(router, 'navigate');
    component.goToPhysicalRobot();
    expect(navigateSpy).toHaveBeenCalledWith(['/robot/physical']);
  });

  it('should navigate to /robot/simulation when goToSimulation is called', () => {
    const navigateSpy = spyOn(router, 'navigate');
    component.goToSimulation();
    expect(navigateSpy).toHaveBeenCalledWith(['/robot/simulation']);
  });

  it('should navigate to /robot/history when goToHistory is called', () => {
    const navigateSpy = spyOn(router, 'navigate');
    component.goToHistory();
    expect(navigateSpy).toHaveBeenCalledWith(['/robot/history']);
  });

  it('should toggle isHelpVisible between true and false when toggleHelp is called', () => {
    expect(component.isHelpVisible).toBeFalse(); // initial value is false
    component.toggleHelp();
    expect(component.isHelpVisible).toBeTrue(); // should be true after the first toggle
    component.toggleHelp();
    expect(component.isHelpVisible).toBeFalse(); // should return to false after second toggle
  });
});
