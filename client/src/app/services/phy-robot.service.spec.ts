import { TestBed } from '@angular/core/testing';
import { RobotService } from './phy-robot.service';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';

describe('RobotService', () => {
  let service: RobotService;
  let httpMock: HttpTestingController;
  const apiUrl = 'http://localhost:3000';

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [RobotService],
    });

    service = TestBed.inject(RobotService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should call identifyRobot with correct URL', () => {
    service.identifyRobot(1).subscribe((response) => {
      expect(response.message).toBe('Identified');
    });

    const req = httpMock.expectOne(`${apiUrl}/robot/1/identify/`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Identified' });
  });

  it('should handle error on identifyRobot', () => {
    service.identifyRobot(1).subscribe({
      next: () => fail('should have failed with 500 error'),
      error: (error) => {
        expect(error.status).toEqual(500);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/robot/1/identify/`);
    req.flush('Internal Server Error', { status: 500, statusText: 'Server Error' });
  });

  it('should call startMission with correct URL', () => {
    service.startMission(2).subscribe((response) => {
      expect(response.message).toBe('Mission Started');
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/2/start`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Mission Started' });
  });

  it('should handle error on startMission', () => {
    service.startMission(2).subscribe({
      next: () => fail('should have failed with 404 error'),
      error: (error) => {
        expect(error.status).toEqual(404);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/2/start`);
    req.flush('Not Found', { status: 404, statusText: 'Not Found' });
  });

  it('should call stopMission with correct URL', () => {
    service.stopMission(1).subscribe((response) => {
      expect(response.message).toBe('Mission Stopped');
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/1/stop`);
    expect(req.request.method).toBe('GET');
    req.flush({ message: 'Mission Stopped' });
  });

  it('should handle error on stopMission', () => {
    service.stopMission(1).subscribe({
      next: () => fail('should have failed with 400 error'),
      error: (error) => {
        expect(error.status).toEqual(400);
      },
    });

    const req = httpMock.expectOne(`${apiUrl}/mission/1/stop`);
    req.flush('Bad Request', { status: 400, statusText: 'Bad Request' });
  });
});
