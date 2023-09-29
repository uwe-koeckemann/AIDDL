val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
    name := "aiddl-core-scala",
    version := "1.1.0",
    versionScheme := Some("early-semver"),

    description := "The AI Domain Definition Language (AIDDL) framework for typing, modeling, " +
      "and creating integrated AI systems.",

    organization := "org.aiddl",
    homepage := Some(url("https://github.com/uwe-koeckemann/AIDDL")),
    scmInfo := Some(
      ScmInfo(
        url("https://github.com/uwe-koeckemann/AIDDL"),
        "https://github.com/uwe-koeckemann/AIDDL.git")),
    developers := List(
      Developer(
        "uwe.koeckemann",
        "Uwe Köckemann",
        "uwe.kockemann@oru.se",
        url("https://github.com/uwe-koeckemann"))),
    licenses += ("MIT", url("https://mit-license.org/")),
    publishMavenStyle := true,
    crossPaths := false,

    isSnapshot := false,
    publishTo := {
      val nexus = "https://s01.oss.sonatype.org/"
      if (isSnapshot.value) Some("snapshots" at nexus + "content/repositories/snapshots")
      else Some("releases" at nexus + "service/local/staging/deploy/maven2")
    },

    scalaVersion := scala3Version,
    parallelExecution := false,

    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",

    scalacOptions ++= Seq("-deprecation", "-feature")
  )
