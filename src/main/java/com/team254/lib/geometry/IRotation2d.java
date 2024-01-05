package com.team254.lib.geometry;

public interface IRotation2d<S> extends State<S> {
  Rotation2d254 getRotation();

  S rotateBy(Rotation2d254 other);

  S mirror();
}
