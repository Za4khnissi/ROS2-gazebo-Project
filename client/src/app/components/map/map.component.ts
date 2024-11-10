// map.component.ts
import { Component, OnInit, ViewChild, ElementRef, AfterViewInit, HostListener } from '@angular/core';
import { WebSocketService } from '@app/services/web-socket.service';

interface Position {
  x: number;
  y: number;
  z: number;
}

interface Orientation {
  x: number;
  y: number;
  z: number;
  w: number;
}

interface RobotPosition {
  robotId: string;
  position: Position;
  orientation: Orientation;
  frame_id: string;
  timestamp: number;
}

interface OccupancyGrid {
  header: {
    seq: number;
    stamp: { sec: number; nsec: number };
    frame_id: string;
  };
  info: {
    map_load_time: { sec: number; nsec: number };
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

@Component({
  selector: 'app-map',
  template: `
    <div #mapContainer class="map-container">
      <canvas #mapCanvas></canvas>
    </div>
  `,
  styles: [`
    .map-container {
      width: 100%;
      height: 400px;
      background: #2d2d2d;
      border-radius: 8px;
      overflow: hidden;
      margin-bottom: 1rem;
      position: relative;
    }
    canvas {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
    }
  `],
  standalone: true
})
export class MapComponent implements OnInit, AfterViewInit {
  @ViewChild('mapCanvas') canvasRef!: ElementRef<HTMLCanvasElement>;
  @ViewChild('mapContainer') containerRef!: ElementRef<HTMLDivElement>;
  private ctx!: CanvasRenderingContext2D;
  private currentMapData: OccupancyGrid | null = null;
  private robotPositions: Map<string, RobotPosition> = new Map();
  private scale: number = 1;
  private robotColors: Map<string, string> = new Map([
    ['limo_105_1', '#FF0000'], // Red
    ['limo_105_2', '#00FF00'], // Green
    ['limo_105_3', '#0000FF'], // Blue
    ['limo_105_4', '#FFFF00']  // Yellow
  ]);

  constructor(private webSocketService: WebSocketService) {}

  ngOnInit() {
    this.webSocketService.listen('map_update').subscribe((mapData: OccupancyGrid) => {
      this.currentMapData = mapData;
      this.drawMap();
    });

    this.webSocketService.listen('robot_position_update').subscribe((positionData: RobotPosition) => {
      this.robotPositions.set(positionData.robotId, positionData);
      this.drawMap();
    });
  }

  ngAfterViewInit() {
    const canvas = this.canvasRef.nativeElement;
    this.ctx = canvas.getContext('2d')!;
  }

  @HostListener('window:resize')
  onResize() {
    if (this.currentMapData) {
      this.drawMap();
    }
  }

  private drawMap() {
    if (!this.currentMapData) return;

    const canvas = this.canvasRef.nativeElement;
    const container = this.containerRef.nativeElement;
    const mapData = this.currentMapData;
    const { width: mapWidth, height: mapHeight } = mapData.info;

    // Get container dimensions
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;

    // Calculate scale to fit map in container while maintaining aspect ratio
    const scaleX = containerWidth / mapWidth;
    const scaleY = containerHeight / mapHeight;
    this.scale = Math.min(scaleX, scaleY) * 0.9; // 0.9 to add some padding

    // Set canvas size based on scaled dimensions
    canvas.width = mapWidth;
    canvas.height = mapHeight;
    canvas.style.width = `${mapWidth * this.scale}px`;
    canvas.style.height = `${mapHeight * this.scale}px`;

    // Create ImageData
    const imageData = this.ctx.createImageData(mapWidth, mapHeight);

    // Fill the image data
    for (let i = 0; i < mapData.data.length; i++) {
      const value = mapData.data[i];
      const idx = i * 4;

      if (value === -1) {
        // Unknown - Gray
        imageData.data[idx] = 128;
        imageData.data[idx + 1] = 128;
        imageData.data[idx + 2] = 128;
        imageData.data[idx + 3] = 255;
      } else if (value === 0) {
        // Free - White
        imageData.data[idx] = 255;
        imageData.data[idx + 1] = 255;
        imageData.data[idx + 2] = 255;
        imageData.data[idx + 3] = 255;
      } else {
        // Occupied - Black
        const intensity = Math.floor(255 * (1 - value / 100));
        imageData.data[idx] = intensity;
        imageData.data[idx + 1] = intensity;
        imageData.data[idx + 2] = intensity;
        imageData.data[idx + 3] = 255;
      }
    }

    // Clear canvas and draw new image
    this.ctx.clearRect(0, 0, mapWidth, mapHeight);
    this.ctx.putImageData(imageData, 0, 0);

    // Draw robots after map
    this.drawRobots();
  }

  private drawRobots() {
    if (!this.currentMapData) return;

    const { resolution, origin } = this.currentMapData.info;

    this.robotPositions.forEach((robotPosition, robotId) => {
      // Convert robot position from world coordinates to pixel coordinates
      const pixelX = (robotPosition.position.x - origin.position.x) / resolution;
      const pixelY = this.currentMapData!.info.height - 
                    (robotPosition.position.y - origin.position.y) / resolution;

      // Draw robot
      this.ctx.save();
      this.ctx.beginPath();
      this.ctx.arc(pixelX, pixelY, 5, 0, 2 * Math.PI);
      this.ctx.fillStyle = this.robotColors.get(robotId) || '#FFFFFF';
      this.ctx.fill();

      // Draw robot direction (using orientation)
      const angle = this.quaternionToYaw(robotPosition.orientation);
      this.ctx.beginPath();
      this.ctx.moveTo(pixelX, pixelY);
      this.ctx.lineTo(
        pixelX + Math.cos(angle) * 10,
        pixelY - Math.sin(angle) * 10
      );
      this.ctx.strokeStyle = this.robotColors.get(robotId) || '#FFFFFF';
      this.ctx.lineWidth = 2;
      this.ctx.stroke();

      // Draw robot ID
      this.ctx.fillStyle = '#FFFFFF';
      this.ctx.font = '12px Arial';
      this.ctx.fillText(robotId, pixelX + 10, pixelY + 10);
      
      this.ctx.restore();
    });
  }

  private quaternionToYaw(orientation: Orientation): number {
    const { x, y, z, w } = orientation;
    return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }
}