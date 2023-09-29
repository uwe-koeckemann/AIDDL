 val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
      name := "aiddl-external-grpc-scala",
      version := "0.2.0",
      versionScheme := Some("early-semver"),
      organization := "org.aiddl",

      description := "Protobuf and gRPC functionality for the AIDDL framework.",

      homepage := Some(url("https://github.com/uwe-koeckemann/AIDDL")),
      scmInfo := Some(ScmInfo(url("https://github.com/uwe-koeckemann/AIDDL"), "https://github.com/uwe-koeckemann/AIDDL.git")),
      developers := List(Developer("uwe.koeckemann", "Uwe Köckemann", "uwe.kockemann@oru.se", url("https://github.com/uwe-koeckemann"))),

      licenses += ("MIT", url("https://mit-license.org/")),
      publishMavenStyle := true,
      crossPaths := false,

      Compile / PB.targets := Seq(
          scalapb.gen() -> (Compile / sourceManaged).value / "scalapb"
      ),

      isSnapshot := true,
      scalaVersion := scala3Version,

      publishTo := {
          val nexus = "https://s01.oss.sonatype.org/"
          if (isSnapshot.value) Some("snapshots" at nexus + "content/repositories/snapshots")
          else Some("releases" at nexus + "service/local/staging/deploy/maven2")
      },

      resolvers += Resolver.mavenLocal,
      libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
      libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
      libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.1.0",
      libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "0.3.0",
      libraryDependencies ++= Seq(
          "io.grpc" % "grpc-netty" % scalapb.compiler.Version.grpcJavaVersion,
          "com.thesamet.scalapb" %% "scalapb-runtime-grpc" % scalapb.compiler.Version.scalapbVersion
      ),

      scalacOptions ++= Seq("-deprecation", "-feature")
  )
