# 3DViewer

[![Documentation](https://softacheck.com/app/repository/faultaddr/3DViewer/documentation/badge)](https://softacheck.com/app/docs/faultaddr/3DViewer/)
[![Softacheck](https://softacheck.com/app/repository/faultaddr/3DViewer/badge)](https://softacheck.com/app/repository/faultaddr/3DViewer/issues)

Thanks For [CloudViewer](https://github.com/nightn/CloudViewer), we use the UI of it, which really save my time. The Initial Idea was to fix some known issue of CloudViewer, But I found that the data structure && code design is a disaster，It is really sucks. So I decided to refactor the code to make a retrofit.

<img  src="https://s1.ax1x.com/2022/06/04/XduaBq.png" width="750" align="center" />

# Description
The 3DViewer is a standalone, tiny, cross-platform point cloud visualization desktop software powered by [PCL](https://github.com/PointCloudLibrary/pcl) and [Qt](https://www.qt.io/).

# Usage
If you want to visualize a 3d model which would like to be pointcloud or mesh. Then u could use the 3DViewer, which designed for light usage for 3D Researchers, The most useful function for me is visualize multiple 3D models in different render window. This makes it easier to compare effects of our deep learning model.

We support various model format:
- ply
- pcd
- stl
- vtk
- obj

# Download
For Linux: (wait for uploading)

For Windows: (is comming)

For Mac: (is comming)

# Compiling
- Linux:
    - git clone --recurse-submodules https://github.com/faultaddr/3DViewer.git
    - install the qt 5.15
    - compile && install vtk 9.1 (using ccmake to choose QT version)
    - compile && install pcl 1.12.1 (using ccmake to choose QT version)
    - mkdir build && cd build && cmake .. && make -j4 && ./src/3DViewer
    - offical docker image: [3DViewer compile docker image](https://hub.docker.com/repository/docker/faultaddr/ubuntu22.04-pcl)

- Windows:
    - I believe that no one wants to use windows, If U are using it now, just wait until the day 3DViewer would compile on it.


# Maintainers

[@faultaddr](https://github.com/faultaddr)

# Star History

[![Star History Chart](https://api.star-history.com/svg?repos=faultaddr/3DViewer&type=Date)](https://star-history.com/#bytebase/star-history&Date)

# Contributing

Feel free to dive in! [open an issue](https://github.com/faultaddr/3DViewer/issues/new) or submit PRs.


3DViewer follows the  [Contributor Covenant](http://contributor-covenant.org/version/1/3/0/) Code of Conduct.


# License

[MIT](LICENSE) © Yunyi Pan
