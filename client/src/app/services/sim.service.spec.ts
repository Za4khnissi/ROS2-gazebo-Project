import { TestBed } from '@angular/core/testing';
import { SimulationService } from './sim.service';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';

describe('SimulationService', () => {
  let service: SimulationService;
  let httpMock: HttpTestingController;
  const apiUrl = 'http://localhost:3000';

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [SimulationService],
    });

    service = TestBed.inject(SimulationService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should call identifyRobot with correct URL', () => {
    service.identifyRobot(3).subscribe((response) => {
      expect(response.message).toBe('Identified');
    });

    const req = httpMock.expectOne(`${apiUrl}/robot/3/identify`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Identified' });
  });

  it('should handle error on identifyRobot', () => {
    service.identifyRobot(3).subscribe({
      next: () => fail('should have failed with 500 error'),
      error: (error) => {
        expect(error.status).toEqual(500);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/robot/3/identify`);
    req.flush('Internal Server Error', { status: 500, statusText: 'Server Error' });
  });

  it('should call startMission with correct URL', () => {
    service.startMission(4).subscribe((response) => {
      expect(response.message).toBe('Mission Started');
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/4/start`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Mission Started' });
  });

  it('should handle error on startMission', () => {
    service.startMission(4).subscribe({
      next: () => fail('should have failed with 404 error'),
      error: (error) => {
        expect(error.status).toEqual(404);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/4/start`);
    req.flush('Not Found', { status: 404, statusText: 'Not Found' });
  });

  it('should call stopMission with correct URL', () => {
    service.stopMission(3).subscribe((response) => {
      expect(response.message).toBe('Mission Stopped');
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/3/stop`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Mission Stopped' });
  });

  it('should handle error on stopMission', () => {
    service.stopMission(3).subscribe({
      next: () => fail('should have failed with 400 error'),
      error: (error) => {
        expect(error.status).toEqual(400);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/3/stop`);
    req.flush('Bad Request', { status: 400, statusText: 'Bad Request' });
  });
});
