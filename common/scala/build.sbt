val scala3Version = "3.3.1"

lazy val root = project
  .in(file("."))
  .settings(
      name := "aiddl-common-scala",
      version := "0.6.1-SNAPSHOT",
      versionScheme := Some("early-semver"),
      organization := "org.aiddl",

      description := "Provides common types and algorithm implementations for the fast prototyping integrative AI systems with the AIDDL framework.",

      homepage := Some(url("https://github.com/uwe-koeckemann/AIDDL")),
      scmInfo := Some(ScmInfo(url("https://github.com/uwe-koeckemann/AIDDL"), "https://github.com/uwe-koeckemann/AIDDL.git")),
      developers := List(Developer("uwe.koeckemann", "Uwe Köckemann", "uwe.kockemann@oru.se", url("https://github.com/uwe-koeckemann"))),

      licenses += ("MIT", url("https://mit-license.org/")),
      publishMavenStyle := true,
      crossPaths := false,

      isSnapshot := true,
      scalaVersion := scala3Version,

      publishTo := {
          val nexus = "https://s01.oss.sonatype.org/"
          if (isSnapshot.value) Some("snapshots" at nexus + "content/repositories/snapshots")
          else Some("releases" at nexus + "service/local/staging/deploy/maven2")
      },
        
      resolvers += Resolver.mavenCentral,
      resolvers += Resolver.mavenLocal,
      
      parallelExecution := false,

      libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.19",
      libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.19" % "test",
      libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.1.2",

      scalacOptions ++= Seq("-deprecation", "-feature")
  )
