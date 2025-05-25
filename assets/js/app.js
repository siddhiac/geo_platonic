// assets/js/app.js

// Import dependencies
import "phoenix_html";
import {Socket} from "phoenix";
import {LiveSocket} from "phoenix_live_view";

//import * as THREE from "https://cdnjs.cloudflare.com/ajax/libs/three.js/r134/three.module.js";
import * as THREE from '../vendor/three';

//import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.134.0/examples/jsm/controls/OrbitControls.js";
import { OrbitControls } from '../vendor/OrbitControls';

// Phoenix LiveView setup
let csrfToken = document.querySelector("meta[name='csrf-token']").getAttribute("content");
let liveSocket = new LiveSocket("/live", Socket, {params: {_csrf_token: csrfToken}});
liveSocket.connect();
window.liveSocket = liveSocket;


document.addEventListener("DOMContentLoaded", function() {
  initializeApp();
});

function initializeApp() {
  // Initialize the globe and map if their containers exist
  const globeContainer = document.getElementById('globe');
  const mapContainer = document.getElementById('map');
  
  if (!globeContainer || !mapContainer) return;

  initDisplayOptionsToggle();  
  initVerticesToggle();
  initMapToggle();
  initGlobeToggle();
  initButtonColors();
  
  // Global state to store current selection and calculated points
  const state = {
    selectedLocation: null,
    antipode: null,
    platonicSolids: {
      tetrahedron: [],
      facing_tetrahedron: [], // Add facing tetrahedron
      cube: [],
      octahedron: [],
      dodecahedron: [],
      icosahedron: []
    },
    displayMode: {
      antipode: false,
      tetrahedron: false,
      facing_tetrahedron: false, // Add facing tetrahedron display mode
      cube: false,
      octahedron: false,
      dodecahedron: false,
      icosahedron: false,
      tetrahedron_pair: false // Add a mode to show both tetrahedrons together
    }
  };
  
  // Initialize Three.js Globe
  const globe = initGlobe(globeContainer);
  
  // Initialize Leaflet Map 
  const map = initMap(mapContainer);
  
  addGlobeZoomControls(globe, globeContainer);

  // Set up event handlers
  setupEventHandlers(globe, map, state);
}

function initGlobe(container) {
  // Three.js setup
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
  
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  container.appendChild(renderer.domElement);
  
  // Add lighting
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambientLight);
  
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
  directionalLight.position.set(5, 3, 5);
  scene.add(directionalLight);
  
  // Create Earth sphere
  const earthRadius = 5;
  const earthGeometry = new THREE.SphereGeometry(earthRadius, 64, 64);
  
  // Load Earth texture
  const textureLoader = new THREE.TextureLoader();
  const earthTexture = textureLoader.load('https://cdn.jsdelivr.net/npm/three-globe/example/img/earth-blue-marble.jpg');
  const earthMaterial = new THREE.MeshPhongMaterial({ 
    map: earthTexture,
    bumpMap: textureLoader.load('https://cdn.jsdelivr.net/npm/three-globe/example/img/earth-topology.png'),
    bumpScale: 0.1,
    specularMap: textureLoader.load('https://cdn.jsdelivr.net/npm/three-globe/example/img/earth-water.png'),
    specular: new THREE.Color('grey')
  });
  
  const earthMesh = new THREE.Mesh(earthGeometry, earthMaterial);
  
  // Create a parent object for Earth and all attached markers
  const earthGroup = new THREE.Group();
  earthGroup.add(earthMesh);
  scene.add(earthGroup);
  
  // Add a subtle starfield background
  const starGeometry = new THREE.BufferGeometry();
  const starCount = 1000;
  const starPositions = new Float32Array(starCount * 3);
  
  for (let i = 0; i < starCount * 3; i += 3) {
    starPositions[i] = (Math.random() - 0.5) * 100;
    starPositions[i + 1] = (Math.random() - 0.5) * 100;
    starPositions[i + 2] = (Math.random() - 0.5) * 100;
  }
  
  starGeometry.setAttribute('position', new THREE.BufferAttribute(starPositions, 3));
  const starMaterial = new THREE.PointsMaterial({
    color: 0xffffff,
    size: 0.1
  });
  
  const starField = new THREE.Points(starGeometry, starMaterial);
  scene.add(starField);
  
  // Set up camera position
  camera.position.z = 15;
  
  // Add orbit controls
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.25;
  controls.screenSpacePanning = false;
  controls.minDistance = 6;
  controls.maxDistance = 30;
  
  // Handle window resize
  window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  });
  
  // IMPORTANT: Change initial rotation state to false
  let isRotating = false;
  
  // Animation loop - modified to respect rotation state
  function animate() {
    requestAnimationFrame(animate);
    
    // Only rotate the globe if isRotating is true
    if (isRotating) {
      earthGroup.rotation.y += 0.0005;
    }
    
    controls.update();
    renderer.render(scene, camera);
  }
  
  animate();
  
  // Return globe API
  return {
    scene,
    camera,
    renderer,
    controls,
    earthMesh,
    earthGroup,  // Export the earth group
    earthRadius,
    

    // Add these two new methods
    toggleRotation: function() {
      isRotating = !isRotating;
      return isRotating;
    },
    
    isRotating: function() {
      return isRotating;
    },


    // Helper to convert lat/lng to 3D coordinates
    latLngToVector3: function(lat, lng, radius) {
      radius = radius || this.earthRadius;
      
      // Convert to radians
      const phi = (90 - lat) * (Math.PI / 180);
      const theta = (lng + 180) * (Math.PI / 180);
      
      // Calculate position
      const x = -radius * Math.sin(phi) * Math.cos(theta);
      const y = radius * Math.cos(phi);
      const z = radius * Math.sin(phi) * Math.sin(theta);
      
      return new THREE.Vector3(x, y, z);
    },
    
    // Add a marker at lat/lng
    addMarker: function(lat, lng, color = 0xff0000, size = 0.1) {
      const position = this.latLngToVector3(lat, lng);
      const markerGeometry = new THREE.SphereGeometry(size, 16, 16);
      const markerMaterial = new THREE.MeshBasicMaterial({ color });
      const marker = new THREE.Mesh(markerGeometry, markerMaterial);
      
      marker.position.set(position.x, position.y, position.z);
      this.earthGroup.add(marker);  // Add to earth group instead of scene
      
      return marker;
    },
    
    // Add a line between two points that follows the Earth's curvature
    addLine: function(point1, point2, color = 0xffffff) {
      // Calculate the number of segments based on the angular distance
      const angle = point1.angleTo(point2);
      // More segments for longer arcs, with a minimum of 12 segments
      const segments = Math.max(12, Math.floor(angle * 20));
      
      // Generate points along the great circle
      const points = [];
      for (let i = 0; i <= segments; i++) {
        // Interpolate between the two points
        const t = i / segments;
        
        // Use spherical interpolation (slerp) for proper great circle path
        const v = new THREE.Vector3().copy(point1).lerp(point2, t).normalize();
        // Scale to Earth radius
        v.multiplyScalar(this.earthRadius * 1.015); // Slightly above surface for visibility
        
        points.push(v);
      }
      
      // Create a curved line along the points
      const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
      const lineMaterial = new THREE.LineBasicMaterial({ 
        color: color,
        linewidth: 2
      });
      const line = new THREE.Line(lineGeometry, lineMaterial);
      
      this.earthGroup.add(line);
      
      return line;
    },


    
    // Focus camera on a specific lat/lng
    focusOn: function(lat, lng) {
      const position = this.latLngToVector3(lat, lng, this.earthRadius * 1.5);
      this.camera.position.set(position.x, position.y, position.z);
      this.controls.update();
    },
    
    // Remove all markers and lines
    clearObjects: function() {
      // Keep track of objects to remove
      const objectsToRemove = [];
      
      // Find all objects in the earth group except the earth mesh itself
      this.earthGroup.children.forEach(child => {
        if (child !== this.earthMesh) {
          objectsToRemove.push(child);
        }
      });
      
      // Remove the objects
      objectsToRemove.forEach(obj => {
        this.earthGroup.remove(obj);
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) obj.material.dispose();
      });
    },
    
    // Also update the addPlatonicSolid function to use curved lines
    addPlatonicSolid: function(vertices, color = 0xffff00) {
      const markers = [];
      const lines = [];
      
      // Add markers for each vertex
      vertices.forEach(vertex => {
        const marker = this.addMarker(vertex.lat, vertex.lng, color, 0.08);
        markers.push(marker);
      });
      
      // Connect vertices with curved lines to form the shape
      for (let i = 0; i < vertices.length; i++) {
        for (let j = i + 1; j < vertices.length; j++) {
          const point1 = this.latLngToVector3(vertices[i].lat, vertices[i].lng);
          const point2 = this.latLngToVector3(vertices[j].lat, vertices[j].lng);
          const line = this.addLine(point1, point2, color);
          lines.push(line);
        }
      }
      
      return { markers, lines };
    }
  };
}

function initMap(container) {
  // Load Leaflet from CDN since we're using a simplified approach
  if (!window.L) {
    console.error('Leaflet not loaded. In a real application, include Leaflet properly.');
    return null;
  }
  
  // Initialize the map
  const map = L.map(container).setView([0, 0], 2);
  
  // Add OpenStreetMap tile layer
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    maxZoom: 18
  }).addTo(map);
  
  // Create layers for different features
  const markersLayer = L.layerGroup().addTo(map);
  
  // Return map API
  return {
    leafletMap: map,
    markersLayer,
    
    // Add a marker to the map
    addMarker: function(lat, lng, options = {}) {
      const marker = L.marker([lat, lng], options).addTo(this.markersLayer);
      return marker;
    },
    
    // Add a line between two points
    addLine: function(point1, point2, options = {}) {
      const line = L.polyline([
        [point1.lat, point1.lng],
        [point2.lat, point2.lng]
      ], options).addTo(this.markersLayer);
      
      return line;
    },
    
    // Clear all markers and lines
    clearLayers: function() {
      this.markersLayer.clearLayers();
    },
    
    // Focus on a location
    focusOn: function(lat, lng, zoom = 5) {
      this.leafletMap.setView([lat, lng], zoom);
    },
    
    // Updated map.addPlatonicSolid function with custom markers
    // This would go in the initMap function where map API is defined

    addPlatonicSolid: function(vertices, options = {}) {
      const markers = [];
      const lines = [];
      
      // Default marker style if not provided
      const markerColor = options.color || '#ff0000';
      const markerSize = options.markerSize || 24; // Increased default size
      const textColor = options.textColor || '#ffffff'; // Default to white text
      
      // Add markers for each vertex with custom styling
      vertices.forEach((vertex, index) => {
        // Create custom divIcon with styling
        const customIcon = L.divIcon({
          className: 'custom-vertex-marker',
          html: `<div style="
            background-color: ${markerColor};
            width: ${markerSize}px;
            height: ${markerSize}px;
            border-radius: 50%;
            border: 2px solid white;
            box-shadow: 0 0 4px rgba(0,0,0,0.5);
            display: flex;
            align-items: center;
            justify-content: center;
            color: ${textColor};
            font-weight: bold;
            font-size: ${markerSize / 2}px;
          ">${index + 1}</div>`,
          iconSize: [markerSize, markerSize],
          iconAnchor: [markerSize/2, markerSize/2]
        });
        
        // Add marker with the custom icon
        const marker = L.marker([vertex.lat, vertex.lng], {
          icon: customIcon,
          title: `Vertex ${index + 1}: ${vertex.lat.toFixed(4)}, ${vertex.lng.toFixed(4)}`
        }).addTo(this.markersLayer);
        
        markers.push(marker);
      });
      
      // Connect vertices with lines
      for (let i = 0; i < vertices.length; i++) {
        for (let j = i + 1; j < vertices.length; j++) {
          const line = L.polyline(
            [
              [vertices[i].lat, vertices[i].lng],
              [vertices[j].lat, vertices[j].lng]
            ], 
            {
              color: options.color || '#ff0000',
              weight: options.weight || 3,
              opacity: options.opacity || 0.85,
              dashArray: options.dashArray || null
            }
          ).addTo(this.markersLayer);
          
          lines.push(line);
        }
      }
      
      return { markers, lines };
    }
  };
}

function setupEventHandlers(globe, map, state) {
  // Fetch references to DOM elements
  const showTetrahedronBtn = document.getElementById('show-tetrahedron');
  const showFacingTetrahedronBtn = document.getElementById('show-facing-tetrahedron');
  const showCubeBtn = document.getElementById('show-cube');
  const showOctahedronBtn = document.getElementById('show-octahedron');
  const showDodecahedronBtn = document.getElementById('show-dodecahedron');
  const showIcosahedronBtn = document.getElementById('show-icosahedron');  
  const showFacingCubeBtn = document.getElementById('show-facing-cube');
  const showFacingOctahedronBtn = document.getElementById('show-facing-octahedron');
  const showFacingDodecahedronBtn = document.getElementById('show-facing-dodecahedron');
  const showFacingIcosahedronBtn = document.getElementById('show-facing-icosahedron');
  const showAllBtn = document.getElementById('show-all');
  const clearAllBtn = document.getElementById('clear-all');
  const inputLat = document.getElementById('input-lat');
  const inputLng = document.getElementById('input-lng');
  const updateLocationBtn = document.getElementById('update-location-btn');
  
  // Info display elements
  const selectedLocationEl = document.getElementById('selected-location');
  const tetrahedronInfoEl = document.querySelector('#tetrahedron-info .vertices-list');
  const facingTetrahedronInfoEl = document.querySelector('#facing-tetrahedron-info .vertices-list');
  const cubeInfoEl = document.querySelector('#cube-info .vertices-list');
  const octahedronInfoEl = document.querySelector('#octahedron-info .vertices-list');
  const dodecahedronInfoEl = document.querySelector('#dodecahedron-info .vertices-list');
  const icosahedronInfoEl = document.querySelector('#icosahedron-info .vertices-list');
  const facingCubeInfoEl = document.querySelector('#facing-cube-info .vertices-list');
  const facingOctahedronInfoEl = document.querySelector('#facing-octahedron-info .vertices-list');
  const facingDodecahedronInfoEl = document.querySelector('#facing-dodecahedron-info .vertices-list');
  const facingIcosahedronInfoEl = document.querySelector('#facing-icosahedron-info .vertices-list');
  
  // Updated rotation toggle button handler
  const toggleRotationBtn = document.getElementById('toggle-rotation');
  if (toggleRotationBtn) {
    // Set the initial button state since we start with rotation off
    toggleRotationBtn.textContent = "Start Rotation";
    toggleRotationBtn.classList.remove('bg-red-600');
    toggleRotationBtn.classList.add('bg-blue-600');
    
    toggleRotationBtn.addEventListener('click', (e) => {
      // Stop event propagation to prevent it bubbling up to the globe container
      e.stopPropagation();
      
      const isRotating = globe.toggleRotation();
      
      // Update button text and style based on state
      if (isRotating) {
        toggleRotationBtn.textContent = "Stop Rotation";
        toggleRotationBtn.classList.remove('bg-blue-600');
        toggleRotationBtn.classList.add('bg-red-600');
      } else {
        toggleRotationBtn.textContent = "Start Rotation";
        toggleRotationBtn.classList.remove('bg-red-600');
        toggleRotationBtn.classList.add('bg-blue-600');
      }
    });
  }

  // Map click handler
  map.leafletMap.on('click', function(e) {
    const lat = e.latlng.lat;
    const lng = e.latlng.lng;
    handleLocationSelection(lat, lng);
  });
  
  // Event handler for the Update button
  updateLocationBtn.addEventListener('click', () => {
    const lat = parseFloat(inputLat.value);
    const lng = parseFloat(inputLng.value);
    
    // Validate input values
    if (isNaN(lat) || isNaN(lng) || lat < -90 || lat > 90 || lng < -180 || lng > 180) {
      alert('Please enter valid coordinates: Latitude (-90 to 90) and Longitude (-180 to 180)');
      return;
    }
    
    // Use the existing handleLocationSelection function
    handleLocationSelection(lat, lng);
  });

  // Handle location selection (from map click)
  function handleLocationSelection(lat, lng) {
    // Clear previous selections
    map.clearLayers();
    globe.clearObjects();
    
    // Update state with new selection
    state.selectedLocation = { lat, lng };
    
    // Update input fields with the new values
    inputLat.value = lat.toFixed(4);
    inputLng.value = lng.toFixed(4);
    
    // Fetch antipode and platonic solids data
    fetchLocationData(lat, lng);
    
    // Add marker on the map
    map.addMarker(lat, lng, {
      title: 'Selected Location',
      icon: L.divIcon({
        className: 'marker-selected',
        html: '📍',
        iconSize: [20, 20],
        iconAnchor: [10, 20]
      })
    });
    
    // Add marker on the globe
    globe.addMarker(lat, lng, 0xff0000, 0.15);
    
    // Focus both views on the selection
    map.focusOn(lat, lng);
    // Slightly delay globe focus for better animation
    setTimeout(() => {
      globe.focusOn(lat, lng);
    }, 100);
  }
  
  // Rest of the setupEventHandlers function remains the same...
  // Fetch location data from server
  function fetchLocationData(lat, lng) {
    // Get CSRF token
    const csrfToken = document.querySelector("meta[name='csrf-token']").getAttribute("content");
    
    // Make API request
    fetch('/calculate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-CSRF-Token': csrfToken
      },
      body: JSON.stringify({ latitude: lat.toString(), longitude: lng.toString() })
    })
    .then(response => response.json())
    .then(data => {
      // Update state with received data
      state.antipode = data.antipode;
      state.platonicSolids = data.platonic_solids;
      
      // Update UI with platonic solids information
      updatePlatonicSolidsInfo(data.platonic_solids);
      
      // Refresh display based on current display mode
      refreshDisplay();
    })
    .catch(error => {
      console.error('Error fetching location data:', error);
    });
  }
  
  // Update platonic solids information
  function updatePlatonicSolidsInfo(platonicSolids) {


    // Helper function to create vertex list HTML
    function createVertexListHTML(vertices) {
      let html = '<ul class="text-sm">';
      vertices.forEach((vertex, index) => {
        html += `<li>Point ${index + 1}: Lat: ${vertex.lat.toFixed(4)}, Lng: ${vertex.lng.toFixed(4)}</li>`;
      });
      html += '</ul>';
      return html;
    }
    
    // Update each solid's info
    tetrahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.tetrahedron);
    facingTetrahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.facing_tetrahedron);
    cubeInfoEl.innerHTML = createVertexListHTML(platonicSolids.cube);
    octahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.octahedron);
    dodecahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.dodecahedron);
    icosahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.icosahedron);
    facingCubeInfoEl.innerHTML = createVertexListHTML(platonicSolids.facing_cube);
    facingOctahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.facing_octahedron);
    facingDodecahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.facing_dodecahedron);
    facingIcosahedronInfoEl.innerHTML = createVertexListHTML(platonicSolids.facing_icosahedron);
  }

  // Rest of the event handlers and functions would remain the same
  // Display antipode on map and globe
  function displayAntipode() {
    if (!state.antipode) return;
    
    // Add antipode marker on map
    map.addMarker(state.antipode.lat, state.antipode.lng, {
      title: 'Antipode',
      icon: L.divIcon({
        className: 'marker-antipode',
        html: '❌',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
      })
    });
    
    // Add antipode marker on globe
    globe.addMarker(state.antipode.lat, state.antipode.lng, 0x0000ff, 0.15);
    
    // Add line through Earth connecting selected point and antipode
    const selectedPos = globe.latLngToVector3(state.selectedLocation.lat, state.selectedLocation.lng);
    const antipodePos = globe.latLngToVector3(state.antipode.lat, state.antipode.lng);
    globe.addLine(selectedPos, antipodePos, 0xffff00);
  }
  
  // Display platonic solid on map and globe
// Updated displayPlatonicSolid function that passes marker options to the map
// This goes in the setupEventHandlers function

function displayPlatonicSolid(solidName, color) {
  if (!state.platonicSolids[solidName] || state.platonicSolids[solidName].length === 0) return;
  
 // Colors for different solids - optimized for high contrast and visibility
  const colors = {
    // Main tetrahedron - vibrant magenta
    tetrahedron: { globe: 0xff00ff, map: '#ff00ff', textColor: '#ffffff' },
    
    // Facing tetrahedron - bright cyan (complementary to magenta)
    facing_tetrahedron: { globe: 0x00ffff, map: '#00ffff', textColor: '#000000' },
    
    // Cube - vivid orange
    cube: { globe: 0xff8000, map: '#ff8000', textColor: '#ffffff' },
    
    // Facing cube - deep blue (complementary to orange)
    facing_cube: { globe: 0x0066ff, map: '#0066ff', textColor: '#ffffff' },
    
    // Octahedron - bright yellow
    octahedron: { globe: 0xffff00, map: '#ffff00', textColor: '#000000' },
    
    // Facing octahedron - royal purple (contrasts with yellow)
    facing_octahedron: { globe: 0x9900ff, map: '#9900ff', textColor: '#ffffff' },
    
    // Dodecahedron - lime green
    dodecahedron: { globe: 0x66ff00, map: '#66ff00', textColor: '#000000' },
    
    // Facing dodecahedron - crimson red (contrasts with green)
    facing_dodecahedron: { globe: 0xff0033, map: '#ff0033', textColor: '#ffffff' },
    
    // Icosahedron - bright white/silver
    icosahedron: { globe: 0xffffff, map: '#ffffff', textColor: '#000000' },
    
    // Facing icosahedron - deep charcoal (contrasts with white)
    facing_icosahedron: { globe: 0x333333, map: '#333333', textColor: '#ffffff' }
  };
  
  const selectedColor = color || colors[solidName];
  
  // Add to globe
  globe.addPlatonicSolid(state.platonicSolids[solidName], selectedColor.globe);
  
  // Map specific options - now with marker configuration
  const mapOptions = {
    color: selectedColor.map,
    weight: 3,         // Increased line weight
    opacity: 0.85,     // Increased opacity
    markerSize: 24,    // Custom marker size
    textColor: selectedColor.textColor, // Text color for the marker
    markerType: getSolidMarkerType(solidName) // See function below
  };
  
  // Add to map with enhanced options
  map.addPlatonicSolid(state.platonicSolids[solidName], mapOptions);
  
  // Update corresponding button color to match
  updateButtonColor(solidName, selectedColor.map, selectedColor.textColor);
}

// Add this new helper function to update button colors
function updateButtonColor(solidName, backgroundColor, textColor) {
  const button = document.getElementById(`show-${solidName}`);
  if (!button) return;
  
  // Remove any existing background color classes
  button.className = button.className.replace(/bg-\w+-\d+/g, '');
  
  // Apply the custom background color directly
  button.style.backgroundColor = backgroundColor;
  button.style.color = textColor;
  
  // Ensure we can still see the button is active
  if (state.displayMode[solidName]) {
    button.classList.add('ring-2');
    button.classList.add('ring-white');
    // Adjust opacity for active state
    button.style.opacity = "0.85";
  } else {
    button.classList.remove('ring-2');
    button.classList.remove('ring-white');
    // Full opacity for inactive state
    button.style.opacity = "1";
  }
}


// Helper function to get different marker styles for different solids
function getSolidMarkerType(solidName) {
  // You can customize markers based on the solid type
  const markerTypes = {
    'tetrahedron': 'circle',
    'facing_tetrahedron': 'circle-outline',
    'cube': 'square',
    'facing_cube': 'square-outline',
    'octahedron': 'diamond',
    'facing_octahedron': 'diamond-outline',
    'dodecahedron': 'pentagon',
    'facing_dodecahedron': 'pentagon-outline',
    'icosahedron': 'triangle',
    'facing_icosahedron': 'triangle-outline'
  };
  
  return markerTypes[solidName] || 'circle'; // Default to circle
}

  
  // Refresh display based on current display mode
  function refreshDisplay() {
    // Clear existing display
    map.clearLayers();
    globe.clearObjects();
    
    // Always show selected location
    if (state.selectedLocation) {
      map.addMarker(state.selectedLocation.lat, state.selectedLocation.lng, {
        title: 'Selected Location',
        icon: L.divIcon({
          className: 'marker-selected',
          html: '📍',
          iconSize: [20, 20],
          iconAnchor: [10, 20]
        })
      });
      
      globe.addMarker(state.selectedLocation.lat, state.selectedLocation.lng, 0xff0000, 0.15);
    }
    
    // Show antipode if enabled
    if (state.displayMode.antipode) {
      displayAntipode();
    }
    
    // Show platonic solids if enabled
    if (state.displayMode.tetrahedron) {
      displayPlatonicSolid('tetrahedron');
    }

    if (state.displayMode.facing_tetrahedron) {
      displayPlatonicSolid('facing_tetrahedron', { globe: 0x00ffff, map: '#00ffff' });
    }
    
    if (state.displayMode.cube) {
      displayPlatonicSolid('cube');
    }
    
    if (state.displayMode.octahedron) {
      displayPlatonicSolid('octahedron');
    }
    
    if (state.displayMode.dodecahedron) {
      displayPlatonicSolid('dodecahedron');
    }
    
    if (state.displayMode.icosahedron) {
      displayPlatonicSolid('icosahedron');
    }

    if (state.displayMode.facing_cube) {
      displayPlatonicSolid('facing_cube');
    }
    
    if (state.displayMode.facing_octahedron) {
      displayPlatonicSolid('facing_octahedron');
    }
    
    if (state.displayMode.facing_dodecahedron) {
      displayPlatonicSolid('facing_dodecahedron');
    }
    
    if (state.displayMode.facing_icosahedron) {
      displayPlatonicSolid('facing_icosahedron');
    }
  }
  
  // Button click handlers with color updates
  showTetrahedronBtn.addEventListener('click', () => {
    state.displayMode.tetrahedron = !state.displayMode.tetrahedron;
    toggleButtonActive(showTetrahedronBtn, state.displayMode.tetrahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0xff00ff, 
      map: '#ff00ff', 
      textColor: '#ffffff'
    };
    updateButtonColor('tetrahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showFacingTetrahedronBtn.addEventListener('click', () => {
    state.displayMode.facing_tetrahedron = !state.displayMode.facing_tetrahedron;
    toggleButtonActive(showFacingTetrahedronBtn, state.displayMode.facing_tetrahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0x00ffff, 
      map: '#00ffff', 
      textColor: '#000000'
    };
    updateButtonColor('facing_tetrahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showCubeBtn.addEventListener('click', () => {
    state.displayMode.cube = !state.displayMode.cube;
    toggleButtonActive(showCubeBtn, state.displayMode.cube);
    
    // Update button color based on current state
    const colors = {
      globe: 0xff8000, 
      map: '#ff8000', 
      textColor: '#ffffff'
    };
    updateButtonColor('cube', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showFacingCubeBtn.addEventListener('click', () => {
    state.displayMode.facing_cube = !state.displayMode.facing_cube;
    toggleButtonActive(showFacingCubeBtn, state.displayMode.facing_cube);
    
    // Update button color based on current state
    const colors = {
      globe: 0x0066ff, 
      map: '#0066ff', 
      textColor: '#ffffff'
    };
    updateButtonColor('facing_cube', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showOctahedronBtn.addEventListener('click', () => {
    state.displayMode.octahedron = !state.displayMode.octahedron;
    toggleButtonActive(showOctahedronBtn, state.displayMode.octahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0xffff00, 
      map: '#ffff00', 
      textColor: '#000000'
    };
    updateButtonColor('octahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showFacingOctahedronBtn.addEventListener('click', () => {
    state.displayMode.facing_octahedron = !state.displayMode.facing_octahedron;
    toggleButtonActive(showFacingOctahedronBtn, state.displayMode.facing_octahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0x9900ff, 
      map: '#9900ff', 
      textColor: '#ffffff'
    };
    updateButtonColor('facing_octahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showDodecahedronBtn.addEventListener('click', () => {
    state.displayMode.dodecahedron = !state.displayMode.dodecahedron;
    toggleButtonActive(showDodecahedronBtn, state.displayMode.dodecahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0x66ff00, 
      map: '#66ff00', 
      textColor: '#000000'
    };
    updateButtonColor('dodecahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showFacingDodecahedronBtn.addEventListener('click', () => {
    state.displayMode.facing_dodecahedron = !state.displayMode.facing_dodecahedron;
    toggleButtonActive(showFacingDodecahedronBtn, state.displayMode.facing_dodecahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0xff0033, 
      map: '#ff0033', 
      textColor: '#ffffff'
    };
    updateButtonColor('facing_dodecahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showIcosahedronBtn.addEventListener('click', () => {
    state.displayMode.icosahedron = !state.displayMode.icosahedron;
    toggleButtonActive(showIcosahedronBtn, state.displayMode.icosahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0xffffff, 
      map: '#ffffff', 
      textColor: '#000000'
    };
    updateButtonColor('icosahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });

  showFacingIcosahedronBtn.addEventListener('click', () => {
    state.displayMode.facing_icosahedron = !state.displayMode.facing_icosahedron;
    toggleButtonActive(showFacingIcosahedronBtn, state.displayMode.facing_icosahedron);
    
    // Update button color based on current state
    const colors = {
      globe: 0x333333, 
      map: '#333333', 
      textColor: '#ffffff'
    };
    updateButtonColor('facing_icosahedron', colors.map, colors.textColor);
    
    refreshDisplay();
  });
  
  // Also update the "Show All" and "Clear All" buttons to handle colors
  showAllBtn.addEventListener('click', () => {
    // Enable all display modes
    Object.keys(state.displayMode).forEach(key => {
      state.displayMode[key] = true;
      
      // Update button states
      const btn = document.getElementById(`show-${key}`);
      if (btn) toggleButtonActive(btn, true);
    });
    
    // Update all button colors according to their respective solids
    updateAllButtonColors();
    
    refreshDisplay();
  });

  clearAllBtn.addEventListener('click', () => {
    // Disable all display modes
    Object.keys(state.displayMode).forEach(key => {
      state.displayMode[key] = false;
      
      // Update button states
      const btn = document.getElementById(`show-${key}`);
      if (btn) toggleButtonActive(btn, false);
    });
    
    // Update all button colors according to their respective solids
    updateAllButtonColors();
    
    refreshDisplay();
  });

  // Helper function to update all button colors at once
  function updateAllButtonColors() {
    const colorMap = {
      tetrahedron: { color: '#ff00ff', textColor: '#ffffff' },
      facing_tetrahedron: { color: '#00ffff', textColor: '#000000' },
      cube: { color: '#ff8000', textColor: '#ffffff' },
      facing_cube: { color: '#0066ff', textColor: '#ffffff' },
      octahedron: { color: '#ffff00', textColor: '#000000' },
      facing_octahedron: { color: '#9900ff', textColor: '#ffffff' },
      dodecahedron: { color: '#66ff00', textColor: '#000000' },
      facing_dodecahedron: { color: '#ff0033', textColor: '#ffffff' },
      icosahedron: { color: '#ffffff', textColor: '#000000' },
      facing_icosahedron: { color: '#333333', textColor: '#ffffff' }
    };
    
    Object.keys(colorMap).forEach(solidName => {
      updateButtonColor(solidName, colorMap[solidName].color, colorMap[solidName].textColor);
    });
  }

  // Modified toggle button active function to maintain color styling
  function toggleButtonActive(button, isActive) {
    if (isActive) {
      button.classList.add('ring-2');
      button.classList.add('ring-white');
      // Add slight transparency for active state
      button.style.opacity = "0.85";
    } else {
      button.classList.remove('ring-2');
      button.classList.remove('ring-white');
      // Full opacity for inactive state
      button.style.opacity = "1";
    }
  }

  // Add the vertex click feature
  const vertexClickFeature = addVertexClickFeature(globe, map);
  
  // Add CSS for clickable vertices
  const style = document.createElement('style');
  style.textContent = `
    .vertices-list ul li {
      cursor: pointer;
      padding: 2px 4px;
      border-radius: 4px;
      transition: background-color 0.2s;
    }
    
    .vertices-list ul li:hover {
      background-color: #f3f4f6;
    }
  `;
  document.head.appendChild(style);
  
  // Hook into the fetch location data function to refresh click handlers
  const originalUpdatePlatonicSolidsInfo = updatePlatonicSolidsInfo;
  updatePlatonicSolidsInfo = function(platonicSolids) {
    // Call the original function
    originalUpdatePlatonicSolidsInfo(platonicSolids);
    
    // Refresh click handlers after a brief delay to ensure DOM is updated
    setTimeout(() => {
      vertexClickFeature.refresh();
    }, 200);
  };
}

function addVertexClickFeature(globe, map) {
  // Helper function to add click handlers to all vertex list items
  function addClickHandlersToVertices() {
    // Find all vertex list items
    const vertexItems = document.querySelectorAll('.vertices-list ul li');
    
    vertexItems.forEach(item => {
      // Add click event listener
      item.addEventListener('click', handleVertexClick);
      
      // Add visual indicator that this is clickable
      item.classList.add('cursor-pointer', 'hover:bg-gray-100');
    });
  }
  
  // Handler for when a vertex is clicked
  function handleVertexClick(event) {
    // Parse coordinates from the text content
    const text = event.target.textContent;
    const latMatch = text.match(/Lat: (-?\d+\.\d+)/);
    const lngMatch = text.match(/Lng: (-?\d+\.\d+)/);
    
    if (latMatch && lngMatch) {
      const lat = parseFloat(latMatch[1]);
      const lng = parseFloat(lngMatch[1]);
      
      // Focus both the globe and map on this location
      focusOnLocation(lat, lng);
      
      // Show detail map for this location
      showDetailMap(lat, lng);
    }
  }
  
  // Focus both the globe and map on the given coordinates
  function focusOnLocation(lat, lng) {
    // Focus the map
    map.focusOn(lat, lng, 5);
    
    // Focus the globe with slight delay for better animation
    setTimeout(() => {
      globe.focusOn(lat, lng);
      
      // Add temporary highlight marker on the globe
      const marker = globe.addMarker(lat, lng, 0xffff00, 0.2);
      
      // Remove the highlight after a few seconds
      setTimeout(() => {
        globe.earthGroup.remove(marker);
        if (marker.geometry) marker.geometry.dispose();
        if (marker.material) marker.material.dispose();
      }, 3000);
    }, 100);
  }
  
  // Create or update the detail map
  function showDetailMap(lat, lng) {
    // Remove any existing detail map
    const existingDetail = document.getElementById('vertex-detail-container');
    if (existingDetail) {
      existingDetail.remove();
    }
    
    // Create the detail container
    const detailContainer = document.createElement('div');
    detailContainer.id = 'vertex-detail-container';
    detailContainer.className = 'mt-6 border-t pt-4';
    
    // Create the content
    detailContainer.innerHTML = `
      <div class="flex justify-between items-center mb-2">
        <h3 class="font-bold text-lg">Selected Vertex Map</h3>
        <button id="close-detail-map" class="px-3 py-1 bg-gray-500 text-white rounded hover:bg-gray-600">
          Close
        </button>
      </div>
      <div class="bg-gray-100 p-4 rounded-lg">
        <div class="flex flex-col md:flex-row gap-4">
          <div class="bg-white p-3 rounded-lg shadow-sm">
            <h4 class="font-semibold mb-2">Coordinates</h4>
            <p class="mb-1"><span class="font-medium">Latitude:</span> ${lat.toFixed(6)}</p>
            <p><span class="font-medium">Longitude:</span> ${lng.toFixed(6)}</p>
          </div>
          <div class="flex-grow">
            <div id="vertex-detail-map" class="h-64 w-full border border-gray-300 rounded-lg"></div>
          </div>
        </div>
      </div>
    `;
    
    // Add to the page after the platonic solids info
    const platonicSolidsInfo = document.getElementById('platonic-solids-info');
    platonicSolidsInfo.after(detailContainer);
    
    // Add close button handler
    document.getElementById('close-detail-map').addEventListener('click', function() {
      detailContainer.remove();
    });
    
    // Initialize the detail map
    initDetailMap(lat, lng);
  }
  
  // Initialize the detail map with Leaflet
  function initDetailMap(lat, lng) {
    // Create a new Leaflet map in the detail container
    const detailMap = L.map('vertex-detail-map').setView([lat, lng], 4);
    
    // Add base map layers
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      maxZoom: 18
    }).addTo(detailMap);
    
    // Add marker for the vertex location
    L.marker([lat, lng], {
      title: 'Vertex Location',
      icon: L.divIcon({
        className: 'marker-vertex',
        html: '📍',
        iconSize: [20, 20],
        iconAnchor: [10, 20]
      })
    }).addTo(detailMap);
    
    // Add a circle for better visibility
    L.circle([lat, lng], {
      color: '#3388ff',
      fillColor: '#3388ff',
      fillOpacity: 0.2,
      radius: 150000 // 150km radius
    }).addTo(detailMap);
    
    // Make sure map renders correctly by triggering a resize after a short delay
    setTimeout(() => {
      detailMap.invalidateSize();
    }, 100);
  }
  
  // Set up mutation observer to detect when vertex lists are updated
  function observeForVertexLists() {
    const observer = new MutationObserver(function(mutations) {
      mutations.forEach(function(mutation) {
        if (mutation.type === 'childList' || mutation.type === 'subtree') {
          // Check if we have vertex lists now
          const vertexLists = document.querySelectorAll('.vertices-list ul');
          if (vertexLists.length > 0) {
            addClickHandlersToVertices();
          }
        }
      });
    });
    
    // Start observing the platonic solids container
    const platonicSolidsInfo = document.getElementById('platonic-solids-info');
    if (platonicSolidsInfo) {
      observer.observe(platonicSolidsInfo, { 
        childList: true,
        subtree: true
      });
    }
  }
  
  // Initialize observer and add initial click handlers if vertices already exist
  observeForVertexLists();
  addClickHandlersToVertices();
  
  // Return public API
  return {
    refresh: addClickHandlersToVertices
  };
}

function initDisplayOptionsToggle() {
  const header = document.getElementById('display-options-header');
  const content = document.getElementById('display-options-content');
  const icon = document.getElementById('toggle-icon');
  
  if (!header || !content || !icon) return;
  
  // Default state is expanded (triangle pointing down)
  icon.style.transform = 'rotate(180deg)';
  
  header.addEventListener('click', function() {
    // Toggle content visibility
    if (content.style.display === 'none') {
      content.style.display = 'flex';
      icon.style.transform = 'rotate(180deg)'; // Triangle pointing down
    } else {
      content.style.display = 'none';
      icon.style.transform = 'rotate(90deg)'; // Triangle pointing right
    }
  });
}

function initVerticesToggle() {
  const header = document.getElementById('vertices-header');
  const content = document.getElementById('vertices-content');
  const icon = document.getElementById('vertices-toggle-icon');
  
  if (!header || !content || !icon) return;
  
  // Default state is expanded (triangle pointing down)
  icon.style.transform = 'rotate(180deg)';
  
  header.addEventListener('click', function() {
    // Toggle content visibility
    if (content.style.display === 'none') {
      content.style.display = 'grid';
      icon.style.transform = 'rotate(180deg)'; // Triangle pointing down
    } else {
      content.style.display = 'none';
      icon.style.transform = 'rotate(90deg)'; // Triangle pointing right
    }
  });
}

function initMapToggle() {
  const header = document.getElementById('map-header');
  const content = document.getElementById('map');
  const icon = document.getElementById('map-toggle-icon');
  
  if (!header || !content || !icon) return;
  
  // Default state is expanded (triangle pointing down)
  icon.style.transform = 'rotate(180deg)';
  
  header.addEventListener('click', function() {
    // Toggle content visibility
    if (content.style.display === 'none') {
      content.style.display = 'flex';
      icon.style.transform = 'rotate(180deg)'; // Triangle pointing down
    } else {
      content.style.display = 'none';
      icon.style.transform = 'rotate(90deg)'; // Triangle pointing right
    }
  });
}

function initGlobeToggle() {
  const header = document.getElementById('globe-header');
  const content = document.getElementById('globe');
  const icon = document.getElementById('globe-toggle-icon');
  
  if (!header || !content || !icon) return;
  
  // Default state is expanded (triangle pointing down)
  icon.style.transform = 'rotate(180deg)';
  
  header.addEventListener('click', function() {
    // Toggle content visibility
    if (content.style.display === 'none') {
      content.style.display = 'flex';
      icon.style.transform = 'rotate(180deg)'; // Triangle pointing down
    } else {
      content.style.display = 'none';
      icon.style.transform = 'rotate(90deg)'; // Triangle pointing right
    }
  });
}

function initButtonColors() {
  const colorMap = {
    tetrahedron: { color: '#ff00ff', textColor: '#ffffff' },
    facing_tetrahedron: { color: '#00ffff', textColor: '#000000' },
    cube: { color: '#ff8000', textColor: '#ffffff' },
    facing_cube: { color: '#0066ff', textColor: '#ffffff' },
    octahedron: { color: '#ffff00', textColor: '#000000' },
    facing_octahedron: { color: '#9900ff', textColor: '#ffffff' },
    dodecahedron: { color: '#66ff00', textColor: '#000000' },
    facing_dodecahedron: { color: '#ff0033', textColor: '#ffffff' },
    icosahedron: { color: '#ffffff', textColor: '#000000' },
    facing_icosahedron: { color: '#333333', textColor: '#ffffff' }
  };
  
  Object.keys(colorMap).forEach(solidName => {
    const button = document.getElementById(`show-${solidName}`);
    if (button) {
      button.style.backgroundColor = colorMap[solidName].color;
      button.style.color = colorMap[solidName].textColor;
    }
  });
}

function addGlobeZoomControls(globe, container) {
  // Create the zoom control container
  const zoomContainer = document.createElement('div');
  zoomContainer.className = 'globe-zoom-controls';
  
  // Create zoom in button
  const zoomInBtn = document.createElement('button');
  zoomInBtn.innerHTML = '+';
  zoomInBtn.className = 'zoom-btn zoom-in';
  zoomInBtn.setAttribute('aria-label', 'Zoom in');
  zoomInBtn.title = 'Zoom in';
  
  // Create zoom out button
  const zoomOutBtn = document.createElement('button');
  zoomOutBtn.innerHTML = '−'; // Using proper minus sign
  zoomOutBtn.className = 'zoom-btn zoom-out';
  zoomOutBtn.setAttribute('aria-label', 'Zoom out');
  zoomOutBtn.title = 'Zoom out';
  
  // Add zoom functionality
  zoomInBtn.addEventListener('click', () => {
    const currentDistance = globe.camera.position.distanceTo(new THREE.Vector3(0, 0, 0));
    const newDistance = Math.max(currentDistance * 0.8, globe.earthRadius * 1.2);
    const direction = new THREE.Vector3().copy(globe.camera.position).normalize();
    globe.camera.position.copy(direction.multiplyScalar(newDistance));
    globe.controls.update();
  });
  
  zoomOutBtn.addEventListener('click', () => {
    const currentDistance = globe.camera.position.distanceTo(new THREE.Vector3(0, 0, 0));
    const newDistance = Math.min(currentDistance * 1.25, globe.earthRadius * 6);
    const direction = new THREE.Vector3().copy(globe.camera.position).normalize();
    globe.camera.position.copy(direction.multiplyScalar(newDistance));
    globe.controls.update();
  });
  
  // Add buttons to container
  zoomContainer.appendChild(zoomInBtn);
  zoomContainer.appendChild(zoomOutBtn);
  
  // Add container to the globe container
  container.style.position = 'relative'; // Ensure proper positioning
  container.appendChild(zoomContainer);
}