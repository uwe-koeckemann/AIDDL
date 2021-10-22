val scala3Version = "3.0.0"

lazy val root = project
  .in(file("."))
  .settings(
    name := "aiddl-common-scala",
    version := "1.0.0",
    organization := "org.aiddl.common",

    isSnapshot := true,
    scalaVersion := scala3Version,

    resolvers += Resolver.mavenLocal,
      parallelExecution := false,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
    libraryDependencies += "org.aiddl.core" % "aiddl-core" % "2.0.0",
    libraryDependencies += "org.aiddl.common" % "aiddl-common" % "2.0.0",
    libraryDependencies += "org.aiddl.core" % "aiddl-core-scala_3" % "1.0.0"
  )
