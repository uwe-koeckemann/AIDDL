val scala3Version = "3.0.0"

lazy val root = project
  .in(file("."))
  .settings(
    name := "planning-n-queens",
    version := "0.1.0",
    organization := "org.aiddl.example",

    isSnapshot := true,
    scalaVersion := scala3Version,

    resolvers += Resolver.mavenLocal,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
    libraryDependencies += "org.aiddl" % "aiddl-core-scala_3" % "2.1.0",
    libraryDependencies += "org.aiddl" % "aiddl-common-scala_3" % "2.1.0"
  )
