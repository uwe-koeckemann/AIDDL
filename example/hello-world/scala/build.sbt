val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
      name := "aiddl-example-hello-world",
      version := "0.1.0-SNAPSHOT",
      versionScheme := Some("early-semver"),
      organization := "org.aiddl",

      description := "A simple Hello World program that uses AIDDL.",

      licenses += ("MIT", url("https://mit-license.org/")),
      publishMavenStyle := true,
      crossPaths := false,

      isSnapshot := true,
      scalaVersion := scala3Version,

      resolvers += Resolver.mavenCentral,
      
      parallelExecution := false,

      libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
      libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
      libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.1.1",
     
      scalacOptions ++= Seq("-deprecation", "-feature")
  )
