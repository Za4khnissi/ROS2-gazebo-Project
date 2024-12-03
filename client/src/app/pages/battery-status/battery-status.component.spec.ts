import { ComponentFixture, TestBed } from '@angular/core/testing';
import { BatteryStatusComponent } from './battery-status.component';
import { WebSocketService } from '@app/services/web-socket.service';
import { of, throwError } from 'rxjs';

describe('BatteryStatusComponent', () => {
  let component: BatteryStatusComponent;
  let fixture: ComponentFixture<BatteryStatusComponent>;
  let webSocketServiceSpy: jasmine.SpyObj<WebSocketService>;

  beforeEach(async () => {
    const socketServiceSpy = jasmine.createSpyObj('WebSocketService', ['listenToBatteryStatus']);

    await TestBed.configureTestingModule({
      imports: [BatteryStatusComponent],
      providers: [
        { provide: WebSocketService, useValue: socketServiceSpy },
      ],
    }).compileComponents();

    webSocketServiceSpy = TestBed.inject(WebSocketService) as jasmine.SpyObj<WebSocketService>;
    fixture = TestBed.createComponent(BatteryStatusComponent);
    component = fixture.componentInstance;
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize with robotId and mode', () => {
    component.robotId = 1;
    component.mode = 'simulation';
    spyOn(console, 'log');

    fixture.detectChanges();

    expect(console.log).toHaveBeenCalledWith('Robot ID:', 1);
  });

  it('should update batteryLevel on receiving new data from WebSocketService', () => {
    component.robotId = 1;
    component.mode = 'physical';
    const mockBatteryLevel = 75;

    webSocketServiceSpy.listenToBatteryStatus.and.returnValue(of(mockBatteryLevel));

    fixture.detectChanges();

    expect(component.batteryLevel).toBe(mockBatteryLevel);
    expect(webSocketServiceSpy.listenToBatteryStatus).toHaveBeenCalledWith(1, 'physical');
  });

  it('should handle WebSocketService errors gracefully', () => {
    component.robotId = 1;
    component.mode = 'simulation';
    spyOn(console, 'error');

    webSocketServiceSpy.listenToBatteryStatus.and.returnValue(throwError(() => new Error('WebSocket error')));

    fixture.detectChanges();

    expect(console.error).toHaveBeenCalledWith('Error receiving battery status:', jasmine.any(Error));
  });

  it('should return correct battery color', () => {
    component.batteryLevel = 60;
    expect(component.getBatteryColor()).toBe('green');

    component.batteryLevel = 30;
    expect(component.getBatteryColor()).toBe('yellow');

    component.batteryLevel = 10;
    expect(component.getBatteryColor()).toBe('red');
  });

  it('should unsubscribe from WebSocketService on destroy', () => {
    component.robotId = 1;
    component.mode = 'physical';
    const mockSubscription = jasmine.createSpyObj('Subscription', ['unsubscribe']);
    webSocketServiceSpy.listenToBatteryStatus.and.returnValue(of(80));
    component['batterySubscription'] = mockSubscription;

    fixture.detectChanges();
    component.ngOnDestroy();

    expect(mockSubscription.unsubscribe).toHaveBeenCalled();
  });
});
