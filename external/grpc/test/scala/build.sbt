val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
    name := "aiddl-external-grpc-scala-test",
    version := "0.1.0-SNAPSHOT",
    organization := "org.aiddl",

    description := "Protobuf and gRPC functionality for the AIDDL framework.",

    homepage := Some(url("https://github.com/uwe-koeckemann/AIDDL")),
    scmInfo := Some(ScmInfo(url("https://github.com/uwe-koeckemann/AIDDL"), "https://github.com/uwe-koeckemann/AIDDL.git")),
    developers := List(Developer("uwe.koeckemann", "Uwe Köckemann", "uwe.kockemann@oru.se", url("https://github.com/uwe-koeckemann"))),

    licenses += ("MIT", url("https://mit-license.org/")),
    publishMavenStyle := true,
    crossPaths := false,

    isSnapshot := true,
    scalaVersion := scala3Version,

    resolvers += Resolver.mavenLocal,
    libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
    libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
    libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.0.0-SNAPSHOT",
    libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "1.0.0-SNAPSHOT",
    libraryDependencies += "org.aiddl" % "aiddl-external-grpc-scala" % "0.1.0-SNAPSHOT",
  )
