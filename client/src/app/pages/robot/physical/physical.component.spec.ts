import { TestBed } from '@angular/core/testing';
import { PhysicalRobotComponent } from './physical.component';
import { RobotService } from '@app/services/phy-robot.service';
import { of, throwError } from 'rxjs';

describe('PhysicalRobotComponent', () => {
  let component: PhysicalRobotComponent;
  let robotServiceSpy: jasmine.SpyObj<RobotService>;

  beforeEach(() => {
    const spy = jasmine.createSpyObj('RobotService', ['identifyRobot', 'startMission', 'stopMission']);

    TestBed.configureTestingModule({
      declarations: [PhysicalRobotComponent],
      providers: [{ provide: RobotService, useValue: spy }],
    }).compileComponents();

    const fixture = TestBed.createComponent(PhysicalRobotComponent);
    component = fixture.componentInstance;
    robotServiceSpy = TestBed.inject(RobotService) as jasmine.SpyObj<RobotService>;
  });

  it('should create the PhysicalRobotComponent', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize robot statuses to "Waiting"', () => {
    expect(component.robot1Status).toBe('Waiting');
    expect(component.robot2Status).toBe('Waiting');
  });

  it('should update robot1Status on successful identifyRobot call for robot 1', () => {
    robotServiceSpy.identifyRobot.and.returnValue(of({ message: 'Identified Robot 1' }));
    component.identifyRobot(1);
    expect(component.robot1Status).toBe('Identified Robot 1');
  });

  it('should update robot2Status on successful identifyRobot call for robot 2', () => {
    robotServiceSpy.identifyRobot.and.returnValue(of({ message: 'Identified Robot 2' }));
    component.identifyRobot(2);
    expect(component.robot2Status).toBe('Identified Robot 2');
  });

  it('should handle error on identifyRobot call', () => {
    spyOn(console, 'error');
    robotServiceSpy.identifyRobot.and.returnValue(throwError(() => new Error('Error')));
    component.identifyRobot(1);
    expect(console.error).toHaveBeenCalledWith('Error identifying robot:', jasmine.any(Error));
  });

  it('should update robot1Status on successful startMission call for robot 1', () => {
    robotServiceSpy.startMission.and.returnValue(of({ message: 'Mission Started Robot 1' }));
    component.startMission(1);
    expect(component.robot1Status).toBe('Mission Started Robot 1');
  });

  it('should update robot2Status on successful startMission call for robot 2', () => {
    robotServiceSpy.startMission.and.returnValue(of({ message: 'Mission Started Robot 2' }));
    component.startMission(2);
    expect(component.robot2Status).toBe('Mission Started Robot 2');
  });

  it('should handle error on startMission call', () => {
    spyOn(console, 'error');
    robotServiceSpy.startMission.and.returnValue(throwError(() => new Error('Error starting mission')));
    component.startMission(1);
    expect(console.error).toHaveBeenCalledWith('Error starting mission:', jasmine.any(Error));
  });

  it('should update robot1Status on successful stopMission call for robot 1', () => {
    robotServiceSpy.stopMission.and.returnValue(of({ message: 'Mission Stopped Robot 1' }));
    component.stopMission(1);
    expect(component.robot1Status).toBe('Mission Stopped Robot 1');
  });

  it('should update robot2Status on successful stopMission call for robot 2', () => {
    robotServiceSpy.stopMission.and.returnValue(of({ message: 'Mission Stopped Robot 2' }));
    component.stopMission(2);
    expect(component.robot2Status).toBe('Mission Stopped Robot 2');
  });

  it('should handle error on stopMission call', () => {
    spyOn(console, 'error');
    robotServiceSpy.stopMission.and.returnValue(throwError(() => new Error('Error stopping mission')));
    component.stopMission(1);
    expect(console.error).toHaveBeenCalledWith('Error stopping mission:', jasmine.any(Error));
  });
});
