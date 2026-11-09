# BS_Robot
Underground cable inspection relies on multiple robotic platforms and the
joint interpretation of visual observations, cable routes, damage locations,
point clouds, positioning data, camera imagery, and subsurface sensing re-
sults. This paper presents BS-Robot, a browser/server software platform for
end-to-end robotic cable inspection. The platform provides robot, crawler,
and manipulator control; real-time telemetry; registration between SLAM
and GPS coordinates; cable-path drawing and import; point-cloud and cam-
era visualization; damage-point marking; evidence capture; inspection-area
selection; and zigzag scan-path generation and dispatch. Its server-side ser-
vices additionally process ground-penetrating radar (GPR) data using con-
figurable preprocessing, Stolt F-K migration, and ONNX-based hyperbola
detection. Inspection paths, damage points, images, annotations, and anal-
ysis results are managed within a unified three-dimensional workspace and
archived for later review. BS-Robot provides an extensible environment for
spatially coherent and reproducible cable-inspection workflows.