import {
    Component,
    OnInit,
    ViewChild,
    ElementRef,
    AfterViewInit,
    OnDestroy,
    HostListener
  } from '@angular/core';
  import { WebSocketService } from '@app/services/web-socket.service';
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
  
  interface Point {
    x: number;
    y: number;
    z: number;
  }
  
  @Component({
    selector: 'app-octomap',
    template: `
      <div #octomapContainer class="octomap-container"></div>
    `,
    styles: [`
      .octomap-container {
        width: 100%;
        height: 400px;
        overflow: hidden;
        position: relative;
      }
    `],
    standalone: true
  })
  export class OctomapComponent implements OnInit, AfterViewInit, OnDestroy {
    @ViewChild('octomapContainer') containerRef!: ElementRef<HTMLDivElement>;
    private scene!: THREE.Scene;
    private camera!: THREE.PerspectiveCamera;
    private renderer!: THREE.WebGLRenderer;
    private controls!: OrbitControls;
    private pointCloudGeometry!: THREE.BufferGeometry;
    private pointCloudMaterial!: THREE.PointsMaterial;
    private pointCloud!: THREE.Points;
  
    constructor(private webSocketService: WebSocketService) {}
  
    ngOnInit() {
      this.webSocketService.listen('octomap_update').subscribe((data: { points: Point[] }) => {
        this.updatePointCloud(data.points);
      });
    }
  
    ngAfterViewInit() {
      this.initThreeJS();
    }
  
    ngOnDestroy() {
      this.renderer.dispose();
      this.controls.dispose();
    }
  
    private initThreeJS() {
      const container = this.containerRef.nativeElement;
  
      // Initialize Three.js Scene
      this.scene = new THREE.Scene();
  
      // Set up the camera
      this.camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
      this.camera.position.set(5, 5, 5);
      this.camera.lookAt(0, 0, 0);
  
      // Set up the renderer
      this.renderer = new THREE.WebGLRenderer({ antialias: true });
      this.renderer.setSize(container.clientWidth, container.clientHeight);
      container.appendChild(this.renderer.domElement);
  
      // Set up OrbitControls for interactivity
      this.controls = new OrbitControls(this.camera, this.renderer.domElement);
      this.controls.mouseButtons = {
        LEFT: THREE.MOUSE.PAN,      // Left click and drag to pan (move without rotating)
        MIDDLE: THREE.MOUSE.DOLLY,   // Middle click to zoom (optional)
        RIGHT: THREE.MOUSE.ROTATE    // Right click and drag to rotate
      };
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.25;
      this.controls.screenSpacePanning = false;
      this.controls.maxDistance = 100;
      
      
  
      // Initialize empty point cloud
      this.pointCloudGeometry = new THREE.BufferGeometry();
      this.pointCloudMaterial = new THREE.PointsMaterial({ vertexColors: true, size: 0.1 });
      this.pointCloud = new THREE.Points(this.pointCloudGeometry, this.pointCloudMaterial);
      this.scene.add(this.pointCloud);
  
      // Start rendering loop
      this.animate();
    }
  
    private animate() {
      requestAnimationFrame(() => this.animate());
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    }
  
    private updatePointCloud(points: Point[]) {
        const positions = new Float32Array(points.length * 3);
        const colors = new Float32Array(points.length * 3);
      
        let minZ = Infinity;
        let maxZ = -Infinity;
      
        // Find the min and max Z values to normalize height-based coloring
        points.forEach(point => {
          minZ = Math.min(minZ, point.z);
          maxZ = Math.max(maxZ, point.z);
        });
      
        const heightRange = maxZ - minZ;
      
        points.forEach((point, i) => {
          positions[i * 3] = point.x;
          positions[i * 3 + 1] = point.y;
          positions[i * 3 + 2] = point.z;
      
          // Normalize the height of the point for the color gradient based on Z-axis
          const heightFactor = heightRange > 0 ? (point.z - minZ) / heightRange : 0;
      
          // Color gradient from green (ground level) to red (higher levels)
          const color = new THREE.Color().setRGB(1 - heightFactor, heightFactor, 0);
          colors[i * 3] = color.r;
          colors[i * 3 + 1] = color.g;
          colors[i * 3 + 2] = color.b;
        });
      
        this.pointCloudGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        this.pointCloudGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
      
        // Adjust the camera target to center on the point cloud in the z-axis
        const midZ = (minZ + maxZ) / 2;
        this.controls.target.set(0, 0, midZ);
      
        this.pointCloudGeometry.attributes['position'].needsUpdate = true;
        this.pointCloudGeometry.attributes['color'].needsUpdate = true;
        this.pointCloudGeometry.computeBoundingBox();
      }
  
    @HostListener('window:resize')
    onResize() {
      const container = this.containerRef.nativeElement;
      this.camera.aspect = container.clientWidth / container.clientHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(container.clientWidth, container.clientHeight);
    }
  }
