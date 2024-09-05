import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PhysicalComponent } from './physical.component';

describe('PhysicalComponent', () => {
  let component: PhysicalComponent;
  let fixture: ComponentFixture<PhysicalComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [PhysicalComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PhysicalComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
