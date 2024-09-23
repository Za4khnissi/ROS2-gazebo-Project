import { TestBed } from '@angular/core/testing';
import { SimulationComponent } from './simulation.component';
import { SimulationService } from '@app/services/sim.service';
import { Router } from '@angular/router';
import { of, throwError } from 'rxjs';

describe('SimulationComponent', () => {
  let component: SimulationComponent;
  let simServiceSpy: jasmine.SpyObj<SimulationService>;
  let routerSpy: jasmine.SpyObj<Router>;

  beforeEach(() => {
    const simSpy = jasmine.createSpyObj('SimulationService', ['identifyRobot', 'startMission', 'stopMission']);
    const routeSpy = jasmine.createSpyObj('Router', ['navigate']);

    TestBed.configureTestingModule({
      declarations: [SimulationComponent],
      providers: [
        { provide: SimulationService, useValue: simSpy },
        { provide: Router, useValue: routeSpy },
      ],
    }).compileComponents();

    const fixture = TestBed.createComponent(SimulationComponent);
    component = fixture.componentInstance;
    simServiceSpy = TestBed.inject(SimulationService) as jasmine.SpyObj<SimulationService>;
    routerSpy = TestBed.inject(Router) as jasmine.SpyObj<Router>;
  });

  it('should create the SimulationComponent', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize robot statuses and simulationStatus correctly', () => {
    expect(component.robot1Status).toBe('Waiting');
    expect(component.robot2Status).toBe('Waiting');
    expect(component.simulationStatus).toBeFalse();
  });

  it('should update robot1Status on successful identifyRobot call for robot 3', () => {
    simServiceSpy.identifyRobot.and.returnValue(of({ message: 'Identified Robot 3' }));
    component.identifyRobot(3);
    expect(component.robot1Status).toBe('Identifying... Identified Robot 3');
  });

  it('should update robot2Status on successful identifyRobot call for robot 4', () => {
    simServiceSpy.identifyRobot.and.returnValue(of({ message: 'Identified Robot 4' }));
    component.identifyRobot(4);
    expect(component.robot2Status).toBe('Identifying... Identified Robot 4');
  });

  it('should handle error on identifyRobot call', () => {
    spyOn(console, 'error');
    simServiceSpy.identifyRobot.and.returnValue(throwError(() => new Error('Error')));
    component.identifyRobot(3);
    expect(console.error).toHaveBeenCalledWith('Error identifying robot:', jasmine.any(Error));
  });

  it('should update robot1Status on successful startMission call for robot 3', () => {
    simServiceSpy.startMission.and.returnValue(of({ message: 'Mission Started Robot 3' }));
    component.startMission(3);
    expect(component.robot1Status).toBe('Mission Started Robot 3');
  });

  it('should update robot2Status on successful startMission call for robot 4', () => {
    simServiceSpy.startMission.and.returnValue(of({ message: 'Mission Started Robot 4' }));
    component.startMission(4);
    expect(component.robot2Status).toBe('Mission Started Robot 4');
  });

  it('should handle error on startMission call', () => {
    spyOn(console, 'error');
    simServiceSpy.startMission.and.returnValue(throwError(() => new Error('Error starting mission')));
    component.startMission(3);
    expect(console.error).toHaveBeenCalledWith('Error starting mission:', jasmine.any(Error));
  });

  it('should update robot1Status on successful stopMission call for robot 3', () => {
    simServiceSpy.stopMission.and.returnValue(of({ message: 'Mission Stopped Robot 3' }));
    component.stopMission(3);
    expect(component.robot1Status).toBe('Mission Stopped Robot 3');
  });

  it('should update robot2Status on successful stopMission call for robot 4', () => {
    simServiceSpy.stopMission.and.returnValue(of({ message: 'Mission Stopped Robot 4' }));
    component.stopMission(4);
    expect(component.robot2Status).toBe('Mission Stopped Robot 4');
  });

  it('should handle error on stopMission call', () => {
    spyOn(console, 'error');
    simServiceSpy.stopMission.and.returnValue(throwError(() => new Error('Error stopping mission')));
    component.stopMission(3);
    expect(console.error).toHaveBeenCalledWith('Error stopping mission:', jasmine.any(Error));
  });
});
