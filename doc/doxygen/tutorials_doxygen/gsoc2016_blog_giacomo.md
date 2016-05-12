
GSoC 2016 Blog - Giacomo {#gsoc2016_blog_giacomo}
====
[TOC]

Framework design and library maintenance  {#gsoc2016_blog_giacomo1}
====

  - [Link to Proposal](https://docs.google.com/document/d/16Wyd0h5b9-7DaG7ZYJT30a2i096krviFUCcDYwg-jZc/edit?usp=sharing) - [Link to GSoC2016 Project Page](https://summerofcode.withgoogle.com/organizations/6007728078061568/#5675882488266752)
 
**About Me**

I finished my **joint master degree** in computer science and networking at Sant'Annas school of advanced studies and the university of Pisa in 2014 with a thesis on the static allocation of real-time OpenMP jobs on multicore machines. The master program was focused on parallel and high performance computing including OpenMP, MPI, Cuda and Tbb.

I started then working as a scholar on the Pelars project (Practice-based Experiential Learning Analytics Research And Support) at the Laboratory of Perceptual Robotics (PERCRO), which is part of the Institute of Communication, Information and Perception Technologies (TECIP) of the Scuola 
Superiore Sant’Anna, Pisa.

In November 2014 I started my PhD in Perceptual Robotics, researching Computer Vision for robotic applications. The first part of my research concerned object recognition algorithms, mainly developing real time solutions; I have worked with different approaches ranging from classical descriptor matching pipelines to machine learning techniques. Both 2D and 3D approaches have been investigated, using different platforms and sensors. I am also active in the research area of RGB-D cameras creating interfaces and testing new sensors.

I stayed as a visiting student for one month at **ETHZ** to investigating object recognition using **Random Forests**, which gave me some great insights on the use of machine learning tecniques in this filed. 

Code is mainly developed using **C**++, **Java**.
I am contributing to the different libraries like **PCL**, **OpenCV** and **libfreenect2**.

-[Link to Github](https://github.com/giacomodabisias)

**General Project Idea**

The concept of this gsoc project is to restructure the OpenDetection Library in order to make it more usable, scalable and documented. In a second phase there could also be the possibility to implement some new features in the library.

The implementation can be divided into several work tasks
- **Task 1** - fix dependencies and modular build : Restructure the Cmake files to obtain a clean and expandable structure. Remove as many dependencies as possible and set build options for the different OpenDetection modules based on the OpenCV/PCL dependencies. 
Depending on OpenCV/PCL version activate/deactivate modules of the library.
- **Task 2** - small tasks : Restructure the code by moving the different files into appropriate folders. Move 3rd party software to git sub modules or downloadable cmake content. Use more templetization and create precompiled versions for basic types. Enhance performances by optimizing the whole code structure. All these steps include the addition of a fixed coding style which can be derived or adopted by an existing one. 
- **Task 3** - moderate task - memory allocation : Restructure memory allocation using shared_pointers where possible to avoid dynamic memory allocation with new. 
- **Task 4** - moderate task - generalized viewer : Create a generalized viewer interface for 2D images and 3D point clouds based on VTK. 
- **Task 5** - moderate task - packaging and cross compilation : Fix the Cmake files in order to make the library cross platform buildable on Linux and Windows systems. Add header precompilation if possible and ProjectConfig.cmake files to to make the library usable by other projects. 
- **Task 6** - moderate - input enhancements : A lot of classes have the possibility to load data from file/path. Create template versions of them in order to let people also load data from vectors/arrays etc.. 
- **Task 8** - small - Documentation and examples : Continue the documentation process and add new examples for the new implemented structures/algorithms. 
- **Task 9** - small task - merge other gsoc16 contributions : Merge changes from other contribution of GSOC. Other projects will be using existing APIs. There might me need to make small changes to fit to the changes made in this project 
- **Task 10** -  moderate task - deb packaging : Create an automated way to generate a deb file for the library so it can be installed through debian packaging system.
