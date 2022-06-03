//
//  ViewController.swift
//  LidarSWIFT
//
//  Created by Georgio Sayegh on 03/06/2022.
//

import UIKit
import ARKit
import RealityKit

class ARViewController: UIViewController {

    @IBOutlet var arView: ARView!
    
    // Cache for 3D text geometries representing the classification values.
    var modelsForClassification: [ARMeshClassification: ModelEntity] = [:]
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        // Prevent the screen from being dimmed to avoid interrupting the AR experience.
        UIApplication.shared.isIdleTimerDisabled = true
    }
    
    override var prefersHomeIndicatorAutoHidden: Bool {
        return true
    }
    
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        //checking if LIDAR is supported
        let supportForSceneReconstruction = ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification)
        
        guard supportForSceneReconstruction else {
            fatalError("Scene Reconsturction isn't supported")
        }
        
        // we set it to false in order to allow mesh with classification
        arView.automaticallyConfigureSession = false
        
        arView.debugOptions.insert(.showSceneUnderstanding)
        
        // Turn on occlusion from the scene reconstruction's mesh.
        arView.environment.sceneUnderstanding.options.insert(.occlusion)
        
        // For performance, disable render options that are not required for this app.
        arView.renderOptions = [.disablePersonOcclusion, .disableDepthOfField, .disableMotionBlur]
        
        let config = ARWorldTrackingConfiguration()
        
        config.sceneReconstruction = .meshWithClassification
        
        config.planeDetection = [.horizontal, .vertical]
        
        config.environmentTexturing = .automatic
        
        
        arView.session.run(config, options: [])
        
        let tap = UITapGestureRecognizer(target: self, action: #selector(didTap))
        arView.addGestureRecognizer(tap)
        
    }
    
    @IBAction func resetConfigurationButton(_ sender: Any) {
        if let config = arView.session.configuration {
            let opts: ARSession.RunOptions = [.resetTracking,
                                              .removeExistingAnchors,
                                              .resetSceneReconstruction]
            arView.session.run(config, options: opts)
        }
    }

    @IBAction func toggleShowHideMeshButton(_ sender: UIButton) {
        let meshShowingOrNot = arView.debugOptions.contains(
                                             .showSceneUnderstanding)
        
        if meshShowingOrNot {
            arView.debugOptions.remove(.showSceneUnderstanding)
            sender.backgroundColor = .red
        } else {
            arView.debugOptions.insert(.showSceneUnderstanding)
            sender.backgroundColor = .green
        }
    }
    
    @objc func didTap(_ sender: UITapGestureRecognizer) {
        // 1. Perform a ray cast against the mesh.
        // Note: Ray-cast option ".estimatedPlane" with alignment ".any" also takes the mesh into account.
        let tapLocation = sender.location(in: arView)
        if let result = arView.raycast(from: tapLocation, allowing: .estimatedPlane, alignment: .any).first {
            // ...
            // 2. Visualize the intersection point of the ray with the real-world surface.
            let resultAnchor = AnchorEntity(world: result.worldTransform)
            resultAnchor.addChild(sphere(radius: 0.01, color: .lightGray))
            arView.scene.addAnchor(resultAnchor, removeAfter: 3)

            // 3. Try to get a classification near the tap location.
            //    Classifications are available per face (in the geometric sense, not human faces).
            nearbyFaceWithClassification(to: result.worldTransform.position) { (centerOfFace, classification) in
                // ...
                DispatchQueue.main.async {
                    // 4. Compute a position for the text which is near the result location, but offset 10 cm
                    // towards the camera (along the ray) to minimize unintentional occlusions of the text by the mesh.
                    let rayDirection = normalize(result.worldTransform.position - self.arView.cameraTransform.translation)
                    let textPositionInWorldCoordinates = result.worldTransform.position - (rayDirection * 0.1)
                    
                    // 5. Create a 3D text to visualize the classification result.
                    let textEntity = self.model(for: classification)

                    // 6. Scale the text depending on the distance, such that it always appears with
                    //    the same size on screen.
                    let raycastDistance = distance(result.worldTransform.position, self.arView.cameraTransform.translation)
                    textEntity.scale = .one * raycastDistance

                    // 7. Place the text, facing the camera.
                    var resultWithCameraOrientation = self.arView.cameraTransform
                    resultWithCameraOrientation.translation = textPositionInWorldCoordinates
                    let textAnchor = AnchorEntity(world: resultWithCameraOrientation.matrix)
                    textAnchor.addChild(textEntity)
                    self.arView.scene.addAnchor(textAnchor, removeAfter: 3)

                    // 8. Visualize the center of the face (if any was found) for three seconds.
                    //    It is possible that this is nil, e.g. if there was no face close enough to the tap location.
                    if let centerOfFace = centerOfFace {
                        let faceAnchor = AnchorEntity(world: centerOfFace)
                        faceAnchor.addChild(self.sphere(radius: 0.01, color: classification.color))
                        self.arView.scene.addAnchor(faceAnchor, removeAfter: 3)
                    }
                }
            }
        }
    }

}

extension ARViewController {
    
    // function that returns the classification of the face near the given location
    func nearbyFaceWithClassification(to location: SIMD3<Float>, completionBlock: @escaping (SIMD3<Float>?, ARMeshClassification) -> Void) {
        guard let frame = arView.session.currentFrame else {
            completionBlock(nil, .none)
            return
        }
    
        var meshAnchors = frame.anchors.compactMap({ $0 as? ARMeshAnchor })
        
        // Sort the mesh anchors by distance to the given location and filter out
        // any anchors that are too far away (4 meters is a safe upper limit).
        let cutoffDistance: Float = 4.0
        meshAnchors.removeAll { distance($0.transform.position, location) > cutoffDistance }
        meshAnchors.sort { distance($0.transform.position, location) < distance($1.transform.position, location) }

        // Perform the search asynchronously in order not to stall rendering.
        DispatchQueue.global().async {
            for anchor in meshAnchors {
                for index in 0..<anchor.geometry.faces.count {
                    // Get the center of the face so that we can compare it to the given location
                    let geometricCenterOfFace = anchor.geometry.centerOf(faceWithIndex: index)
                    
                    // Conver the face's center to world coordinates
                    var centerLocalTransform = matrix_identity_float4x4
                    centerLocalTransform.columns.3 = SIMD4<Float>(geometricCenterOfFace.0, geometricCenterOfFace.1, geometricCenterOfFace.2, 1)
                    let centerWorldPosition = (anchor.transform * centerLocalTransform).position
                    
                    // We're intersted in a classification sufficiently close to the given location - say 5cm close
                    let distanceToFace = distance(centerWorldPosition, location)
                    if distanceToFace <= 0.05 {
                        // Get semantic classification of the face and call completion handler
                        let classification: ARMeshClassification = anchor.geometry.classificationOf(faceWithIndex: index)
                        completionBlock(centerWorldPosition, classification)
                        return
                    }
                }
            }
            
            // If no results found
            completionBlock(nil, .none)
        }

    }
    
    // Creating a Text Model for classification and storing it in cache
    func model(for classification: ARMeshClassification) -> ModelEntity {
        // Return cached model if available
        if let model = modelsForClassification[classification] {
            model.transform = .identity
            return model.clone(recursive: true)
        }
        
        // Generate 3D text for the classification
        let lineHeight: CGFloat = 0.05
        let font = MeshResource.Font.systemFont(ofSize: lineHeight)
        let textMesh = MeshResource.generateText(classification.description, extrusionDepth: Float(lineHeight * 0.1), font: font)
        let textMaterial = SimpleMaterial(color: classification.color, isMetallic: true)
        let model = ModelEntity(mesh: textMesh, materials: [textMaterial])
        // Move text geometry to the left so that its local origin is in the center
        model.position.x -= model.visualBounds(relativeTo: nil).extents.x / 2
        // Add model to cache
        modelsForClassification[classification] = model
        return model
    }
    
    // function that creates a sphere entity
    func sphere(radius: Float, color: UIColor) -> ModelEntity {
        let sphere = ModelEntity(mesh: .generateSphere(radius: radius), materials: [SimpleMaterial(color: color, isMetallic: false)])
        // Move sphere up by half its diameter so that it does not intersect with the mesh
        sphere.position.y = radius
        return sphere
    }
}
