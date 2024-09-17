import { TestBed } from '@angular/core/testing';

import { RobotService } from '@app/services/phy-robot.service';

describe('ServicesService', () => {
  let service: RobotService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(RobotService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
