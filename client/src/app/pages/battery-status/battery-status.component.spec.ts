import { ComponentFixture, TestBed } from '@angular/core/testing';
import { BatteryStatusComponent } from './battery-status.component';
import { WebSocketService } from '@app/services/web-socket.service';
import { of, Subject } from 'rxjs';

describe('BatteryStatusComponent', () => {
  let component: BatteryStatusComponent;
  let fixture: ComponentFixture<BatteryStatusComponent>;
  let mockWebSocketService: jasmine.SpyObj<WebSocketService>;
  let batterySubject: Subject<number>;

  beforeEach(async () => {
    batterySubject = new Subject<number>();

    mockWebSocketService = jasmine.createSpyObj('WebSocketService', ['listenToBatteryStatus']);
    mockWebSocketService.listenToBatteryStatus.and.returnValue(batterySubject.asObservable());

    await TestBed.configureTestingModule({
      imports: [BatteryStatusComponent],
      providers: [{ provide: WebSocketService, useValue: mockWebSocketService }],
    }).compileComponents();

    fixture = TestBed.createComponent(BatteryStatusComponent);
    component = fixture.componentInstance;

    component.robotId = 1;
    component.mode = 'simulation';
    fixture.detectChanges();
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });

  it('should subscribe to battery updates on init', () => {
    expect(mockWebSocketService.listenToBatteryStatus).toHaveBeenCalledWith(1, 'simulation');
  });

  it('should update batteryLevel to the new minimum value', () => {
    batterySubject.next(80);
    expect(component.batteryLevel).toBe(80);

    batterySubject.next(70);
    expect(component.batteryLevel).toBe(70);

    batterySubject.next(90); // Should not increase the batteryLevel
    expect(component.batteryLevel).toBe(70);
  });

  it('should return the correct battery color', () => {
    component.batteryLevel = 60;
    expect(component.getBatteryColor()).toBe('green');

    component.batteryLevel = 40;
    expect(component.getBatteryColor()).toBe('yellow');

    component.batteryLevel = 10;
    expect(component.getBatteryColor()).toBe('red');
  });

  it('should unsubscribe from battery updates on destroy', () => {
    const unsubscribeSpy = spyOn(component['batterySubscription'], 'unsubscribe');
    component.ngOnDestroy();
    expect(unsubscribeSpy).toHaveBeenCalled();
  });

  it('should log an error if battery update throws an error', () => {
    const consoleErrorSpy = spyOn(console, 'error');
    const testError = new Error('Test error');

    batterySubject.error(testError);

    expect(consoleErrorSpy).toHaveBeenCalledWith('Error receiving battery status:', testError);
  });
});
