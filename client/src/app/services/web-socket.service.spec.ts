import { TestBed } from '@angular/core/testing';
import { WebSocketService } from './web-socket.service';
import { io, Socket } from 'socket.io-client';

describe('WebSocketService', () => {
  let service: WebSocketService;
  let mockSocket: Partial<Socket>;

  beforeEach(() => {
    // Mock des méthodes de `Socket`
    mockSocket = {
      on: jasmine.createSpy('on'),
      emit: jasmine.createSpy('emit'),
      disconnect: jasmine.createSpy('disconnect'),
    };

    // Mock de la fonction `io` pour retourner le mock de `Socket`
    spyOn<any>(io, 'default').and.returnValue(mockSocket as Socket);

    TestBed.configureTestingModule({
      providers: [WebSocketService],
    });

    service = TestBed.inject(WebSocketService);
  });

  afterEach(() => {
    // Réinitialisation des appels mockés après chaque test
    (mockSocket.on as jasmine.Spy).calls.reset();
    (mockSocket.emit as jasmine.Spy).calls.reset();
  });

  it('devrait être créé', () => {
    expect(service).toBeTruthy();
  });

  it('devrait émettre un événement avec emit()', () => {
    const eventName = 'testEvent';
    const eventData = { message: 'Hello' };

    service.emit(eventName, eventData);

    expect(mockSocket.emit).toHaveBeenCalledWith(eventName, eventData);
  });

  it('devrait écouter un événement avec listen()', (done: DoneFn) => {
    const eventName = 'testEvent';
    const mockData = { data: 'testData' };

    (mockSocket.on as jasmine.Spy).and.callFake((event, callback) => {
      if (event === eventName) {
        callback(mockData);
      }
    });

    service.listen(eventName).subscribe((data) => {
      expect(data).toEqual(mockData);
      done();
    });
  });

  it('devrait écouter les mises à jour de la batterie avec listenToBatteryStatus()', (done: DoneFn) => {
    const robotId = 1;
    const mode = 'simulation';
    const batteryLevel = 80;

    const mockData = { batteryLevel };

    (mockSocket.on as jasmine.Spy).and.callFake((event, callback) => {
      if (event === 'batteryUpdate') {
        callback(mockData);
      }
    });

    service.listenToBatteryStatus(robotId, mode).subscribe((level) => {
      expect(mockSocket.emit).toHaveBeenCalledWith('clientMessage', { robotId, mode });
      expect(level).toEqual(batteryLevel);
      done();
    });
  });

  it('devrait gérer une mise à jour sans niveau de batterie défini', (done: DoneFn) => {
    const robotId = 2;
    const mode = 'physical';

    const mockData = { invalidData: 'error' };

    (mockSocket.on as jasmine.Spy).and.callFake((event, callback) => {
      if (event === 'batteryUpdate') {
        callback(mockData);
      }
    });

    service.listenToBatteryStatus(robotId, mode).subscribe((level) => {
      expect(level).toEqual(0); // Niveau de batterie par défaut
      done();
    });
  });
});
