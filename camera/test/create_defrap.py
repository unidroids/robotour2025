import numpy as np

d = np.load("camera_calibration.npz")
K = d["cameraMatrix"]
D = d["distCoeffs"].flatten()

with open("dewarp.cfg", "w") as f:
    f.write("<dewarp version=\"1.0\">\n")
    f.write("  <camera_matrix>\n")
    f.write(f"    {K[0,0]:.3f} {K[0,1]:.3f} {K[0,2]:.3f}\n")
    f.write(f"    {K[1,0]:.3f} {K[1,1]:.3f} {K[1,2]:.3f}\n")
    f.write(f"    {K[2,0]:.3f} {K[2,1]:.3f} {K[2,2]:.3f}\n")
    f.write("  </camera_matrix>\n")
    f.write("  <distortion type=\"radial-tangential\">\n")
    f.write("    " + " ".join([f"{x:.3f}" for x in D[:5]]) + "\n")
    f.write("  </distortion>\n")
    f.write("</dewarp>\n")
