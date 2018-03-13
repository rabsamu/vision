# Stryke Force Quickstart

This repository gives students a quick start to setting up a FRC robot project in Java. We use powerful [GradleRIO](https://github.com/Open-RIO/GradleRIO) to build our code.

## Prerequisites

Do these steps once to set up your development computer for FRC Java programming.

Go to [www.putty.org](http://www.putty.org)  click the download here button, install the appropriate (32 vs. 64) version. Open and complete the installation wizard with the default settings.

### Java Development Kit

Check to see if Java 8 Java Development Kit (JDK) is installed by running `javac -version` in a terminal window. If the JDK is installed you will see something similar to:

```
$ javac -version
javac 1.8.0_77
```

You can download the JDK [here](http://www.oracle.com/technetwork/java/javase/downloads/index.html). Select the 64-bit JDK appropriate for your development computer.

### IntelliJ IDEA

Download and install the IntelliJ IDEA development environment from [here](https://www.jetbrains.com/idea/download/).

## Setup

Follow the steps below to use our quickstart repository.

1. Download an [archive](https://github.com/strykeforce/quickstart/releases) or fork this repo. Forking will let you practice making changes and committing them to your own GitHub account.
2. Clone the repository you just forked to your development computer.
3. Open a terminal window and change directory to the `quickstart` directory you just cloned to your development computer.
4. Run `./gradlew idea` on macOS/Linux and `gradlew idea` on Windows to set up the project for IntelliJ. The first time you run this GradleRIO will download a bunch of dependencies.
5. Open `quickstart.ipr` to open the project in Intellij.

This repository has a small "Hello World" program to test your installation with. Go to the quickstart directory in a terminal window and run:

```sh
# lines starting with "$" are the commands you type
$ gradlew build
...some stuff
BUILD SUCCESSFUL

$ java -classpath build/classes/java/main org.strykeforce.quickstart.Robot
Hello from Quickstart!
```
