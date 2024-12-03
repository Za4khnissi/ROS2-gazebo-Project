import { TestBed, fakeAsync, tick } from '@angular/core/testing';
import { PhysicalRobotComponent } from './physical.component';
import { RobotService } from '@app/services/robot.service';
import { WebSocketService } from '@app/services/web-socket.service';
import { Router } from '@angular/router';
import { of, throwError } from 'rxjs';
import { ChangeDetectorRef } from '@angular/core';

describe('PhysicalRobotComponent', () => {
  let component: PhysicalRobotComponent;
  let robotServiceSpy: jasmine.SpyObj<RobotService>;
  let webSocketServiceSpy: jasmine.SpyObj<WebSocketService>;
  let routerSpy: jasmine.SpyObj<Router>;

  beforeEach(() => {
    const robotSpy = jasmine.createSpyObj('RobotService', [
      'identifyRobot',
      'startMission',
      'stopMission',
      'returnFromMission',
      'getOldLogs',
    ]);
    const webSocketSpy = jasmine.createSpyObj('WebSocketService', ['listen']);
    const routerSpyObj = jasmine.createSpyObj('Router', ['navigate']);

    TestBed.configureTestingModule({
      imports: [PhysicalRobotComponent],
      providers: [
        { provide: RobotService, useValue: robotSpy },
        { provide: WebSocketService, useValue: webSocketSpy },
        { provide: Router, useValue: routerSpyObj },
        ChangeDetectorRef,
      ],
    }).compileComponents();

    const fixture = TestBed.createComponent(PhysicalRobotComponent);
    component = fixture.componentInstance;
    robotServiceSpy = TestBed.inject(RobotService) as jasmine.SpyObj<RobotService>;
    webSocketServiceSpy = TestBed.inject(WebSocketService) as jasmine.SpyObj<WebSocketService>;
    routerSpy = TestBed.inject(Router) as jasmine.SpyObj<Router>;
  });

  it('should create the PhysicalRobotComponent', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize robot statuses to "Waiting"', () => {
    expect(component.robot1Status).toBe('Waiting');
    expect(component.robot2Status).toBe('Waiting');
  });

  describe('WebSocket integration', () => {
    it('should handle syncUpdate event and update robot status', fakeAsync(() => {
      const mockData = { robot: '1', event: 'mission_started' };
      webSocketServiceSpy.listen.and.returnValue(of(mockData));

      component.ngOnInit();
      tick(); // Simuler le passage du temps pour les tâches asynchrones
      expect(webSocketServiceSpy.listen).toHaveBeenCalledWith('syncUpdate');
      expect(component.robot1Status).toBe('Moving');
    }));

    it('should handle unknown event in WebSocket message', fakeAsync(() => {
      const mockData = { robot: '1', event: 'unknown_event' };
      webSocketServiceSpy.listen.and.returnValue(of(mockData));

      component.ngOnInit();
      tick();
      expect(component.robot1Status).toBe('Waiting');
    }));
  });

  describe('identifyRobot', () => {
    it('should call identifyRobot and log success', fakeAsync(() => {
      // Configuration pour simuler une réponse réussie
      robotServiceSpy.identifyRobot.and.returnValue(of({ message: 'Identified' }));
      
      // Spy sur console.log
      const logSpy = spyOn(console, 'log');

      // Appel de la méthode
      component.identifyRobot(1);
      tick(); // Simule le passage du temps pour résoudre l'observable

      // Vérifications
      expect(robotServiceSpy.identifyRobot).toHaveBeenCalledWith(1);
      expect(logSpy).toHaveBeenCalledWith('Identified');
    }));

    it('should handle error on identifyRobot call', fakeAsync(() => {
      // Spy sur console.error
      const errorSpy = spyOn(console, 'error');
      
      // Configuration pour simuler une erreur
      robotServiceSpy.identifyRobot.and.returnValue(
        throwError(() => new Error('Error identifying robot'))
      );

      // Appel de la méthode
      component.identifyRobot(1);
      tick();

      // Vérifications
      expect(robotServiceSpy.identifyRobot).toHaveBeenCalledWith(1);
      expect(errorSpy).toHaveBeenCalledWith(
        'Error identifying robot:',
        jasmine.any(Error)
      );
    }));
  });

  describe('startMission', () => {
    it('should call startMission and log success', fakeAsync(() => {
      robotServiceSpy.startMission.and.returnValue(of({ message: 'Mission Started' }));
      spyOn(console, 'log');
      component.startMission(1);
      tick();
      expect(robotServiceSpy.startMission).toHaveBeenCalledWith(1);
      expect(console.log).toHaveBeenCalledWith('Mission Started');
    }));

    it('should handle error on startMission call', fakeAsync(() => {
      spyOn(console, 'error');
      robotServiceSpy.startMission.and.returnValue(throwError(() => new Error('Error starting mission')));
      component.startMission(1);
      tick();
      expect(console.error).toHaveBeenCalledWith('Error starting mission:', jasmine.any(Error));
    }));
  });

  describe('stopMission', () => {
    it('should call stopMission and log success', fakeAsync(() => {
      robotServiceSpy.stopMission.and.returnValue(of({ message: 'Mission Stopped' }));
      spyOn(console, 'log');
      component.stopMission(1);
      tick();
      expect(robotServiceSpy.stopMission).toHaveBeenCalledWith(1);
      expect(console.log).toHaveBeenCalledWith('Mission Stopped');
    }));

    it('should handle error on stopMission call', fakeAsync(() => {
      spyOn(console, 'error');
      robotServiceSpy.stopMission.and.returnValue(throwError(() => new Error('Error stopping mission')));
      component.stopMission(1);
      tick();
      expect(console.error).toHaveBeenCalledWith('Error stopping mission:', jasmine.any(Error));
    }));
  });

  describe('toggleOldLogs', () => {
    it('should fetch and display old logs when showOldLogs is false', fakeAsync(() => {
      const mockMissions = [{ id: 1, logs: [] }];
      robotServiceSpy.getOldLogs.and.returnValue(of(mockMissions));

      component.toggleOldLogs();
      tick();
      expect(robotServiceSpy.getOldLogs).toHaveBeenCalled();
      expect(component.missions.length).toBe(1);
      expect(component.showOldLogs).toBeTrue();
    }));

    it('should clear missions when showOldLogs is true', fakeAsync(() => {
      component.showOldLogs = true;
      component.missions = [{ id: 1, logs: [] }];
      component.toggleOldLogs();
      tick();
      expect(component.missions.length).toBe(0);
      expect(component.showOldLogs).toBeFalse();
    }));
  });

  describe('toggleMissionLogs', () => {
    it('should toggle expanded property of a mission', () => {
      const mockMission = { id: 1, expanded: false };
      component.toggleMissionLogs(mockMission);
      expect(mockMission.expanded).toBeTrue();

      component.toggleMissionLogs(mockMission);
      expect(mockMission.expanded).toBeFalse();
    });
  });
});
