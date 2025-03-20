val scala3Version = "3.3.4"

lazy val root = project
  .in(file("."))
  .settings(
    name := "planning-n-queens",
    version := "0.1.0-SNAPSHOT",
    organization := "org.aiddl",

    isSnapshot := true,
    scalaVersion := scala3Version,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.19",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.19" % "test",
    libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.1.2",
    libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "0.5.1"
  )
