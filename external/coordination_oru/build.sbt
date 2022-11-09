val scala3Version = "3.1.2"

lazy val root = project
  .in(file("."))
  .settings(
      name := "aiddl-external-coordination_oru",
      version := "0.1.0-SNAPSHOT",
      organization := "org.aiddl",

      isSnapshot := true,
      scalaVersion := scala3Version,
      crossPaths := false,

      resolvers += Resolver.mavenLocal,
      resolvers += Resolver.jcenterRepo,
      resolvers += "jitpack Repo" at "https://jitpack.io/",
      resolvers += "rosjava Repo" at "https://github.com/rosjava/rosjava_mvn_repo/raw/master/",

      libraryDependencies += "org.scalactic" %% "scalactic" % "3.2.9",
      libraryDependencies += "org.scalatest" %% "scalatest" % "3.2.9" % "test",
      libraryDependencies += "org.aiddl" % "aiddl-core-scala" % "1.0.0-SNAPSHOT",
      libraryDependencies += "org.aiddl" % "aiddl-common-scala" % "1.0.0-SNAPSHOT",
      libraryDependencies += "se.oru.coordination" % "coordination_oru" % "0.6.3"
  )
