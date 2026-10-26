# SocialNav

This repository contains the MATLAB implementations corresponding to the following research works:

1. **Topo-Geometric Coordination including Door Crossing (Paper 1):** Focuses on the coordination of geometric and topological levels to enable a bimanual mobile robot to successfully navigate through doors.
2. **Social Navigation in Wide Environments (Paper 2):** Implements sensor fusion techniques for social navigation on mobile robots using the Fast Marching Square method and Gaussian Mixture Models.

---

## ⚙️ Dependencies

* **MATLAB**
* **Fast Marching Toolbox:** Available on the MATLAB Central File Exchange: [Fast Marching Toolbox](https://es.mathworks.com/matlabcentral/fileexchange/6110-toolbox-fast-marching)

---

## 🚀 Main Scripts

* **Real Robot (ADAM):** `NAVEGACIONpasoPuertas.m` (for physical bimanual mobile robot execution).
* **Simulation:** `Turtlebot3_FMSquare.m` (for simulated environments).

---

## 📄 Citation

If you use this code or work in your research, please cite the corresponding papers:

```bibtex
@inproceedings{mora2022coordinacion,
  title     = {Coordinaci{\'o}n de niveles geom{\'e}trico y topol{\'o}gico para el paso de puertas de un robot m{\'o}vil bimanipulador},
  author    = {Mora, Alicia and Prados, Adri{\'a}n and Casta{\~n}o, Ram{\'o}n I Barber and Lorente, Luis Enrique Moreno},
  booktitle = {XLIII Jornadas de Autom{\'a}tica: libro de actas: 7, 8 y 9 de septiembre de 2022, Logro{\~n}o (La Rioja)},
  pages     = {770--776},
  year      = {2022},
  organization = {Servizo de Publicaci{\'o}ns}
}

@article{mora2022sensor,
  title     = {Sensor fusion for social navigation on a mobile robot based on fast marching square and gaussian mixture model},
  author    = {Mora, Alicia and Prados, Adrian and Mendez, Alberto and Barber, Ramon and Garrido, Santiago},
  journal   = {Sensors},
  volume    = {22},
  number    = {22},
  pages     = {8728},
  year      = {2022},
  publisher = {MDPI}
}
