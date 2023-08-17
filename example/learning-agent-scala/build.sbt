val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
      name := "aiddl-example-learning-agent-scala",
      version := "0.1.0-SNAPSHOT",
      versionScheme := Some("early-semver"),
      organization := "org.aiddl",

      description := "Two integrations between planning and learning.",

      licenses += ("MIT", url("https://mit-license.org/")),
      publishMavenStyle := true,
      crossPaths := false,

      isSnapshot := true,
      scalaVersion := scala3Version,

      resolvers += Resolver.mavenLocal,
      resolvers += Resolver.mavenCentral,
      
      parallelExecution := false,

      libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
      libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
      libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.0.0",
      libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "0.1.1",
     
      scalacOptions ++= Seq("-deprecation", "-feature")
  )
