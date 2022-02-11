val scala3Version = "3.0.0"

lazy val root = project
  .in(file("."))
  .settings(
    name := "aiddl-common-scala",
    version := "1.0.0",
    organization := "org.aiddl",

    isSnapshot := true,
    scalaVersion := scala3Version,

    resolvers += Resolver.mavenLocal,
    parallelExecution := false,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
    libraryDependencies += "org.aiddl" % "aiddl-core-scala_3" % "1.0.0"
  )
