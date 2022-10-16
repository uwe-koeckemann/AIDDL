val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
    name := "planning-n-queens",
    version := "0.1.0-SNAPSHOT",
    organization := "org.aiddl",

    isSnapshot := true,
    scalaVersion := scala3Version,

    resolvers += Resolver.mavenLocal,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
    libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.0.0-SNAPSHOT",
    libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "1.0.0-SNAPSHOT"
  )
