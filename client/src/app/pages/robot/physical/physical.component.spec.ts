import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PhysicalRobotComponent } from './physical.component';

describe('PhysicalComponent', () => {
  let component: PhysicalRobotComponent;
  let fixture: ComponentFixture<PhysicalRobotComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [PhysicalRobotComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PhysicalRobotComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
