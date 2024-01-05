package com.team254.lib.spline;

import com.team254.lib.geometry.*;
import com.team254.lib.trajectory.TrajectoryPoint;
import com.team254.lib.trajectory.TrajectoryTest;
import com.team254.lib.util.Util;
import java.util.List;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class SplineGeneratorTest {
  public static final double kTestEpsilon = Util.kEpsilon;

  @Test
  public void test() {
    // Create the test spline
    Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d254());
    Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d254(1, -5, true));
    Spline s = new QuinticHermiteSpline(p1, p2);
    List<Rotation2d254> headings =
        List.of(Rotation2d254.fromDegrees(0), Rotation2d254.fromDegrees(90));

    List<TrajectoryPoint<Pose2dWithCurvature, Rotation2d254>> samples =
        SplineGenerator.parameterizeSpline(s, headings);

    double arclength = 0;
    Rotation2d254 cur_heading = Rotation2d254.identity();
    Pose2dWithCurvature cur_pose = samples.get(0).state();
    for (TrajectoryPoint<Pose2dWithCurvature, Rotation2d254> point : samples) {
      Pose2dWithCurvature sample = point.state();
      final Twist2d t = Pose2d.log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
      arclength += t.dx;
      cur_pose = sample;
      cur_heading = point.heading();
    }

    Assert.assertEquals(cur_pose.getTranslation().x(), 15.0, kTestEpsilon);
    Assert.assertEquals(cur_pose.getTranslation().y(), 10.0, kTestEpsilon);
    Assert.assertEquals(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
    Assert.assertEquals(arclength, 23.17291953186379, kTestEpsilon);
    Assert.assertEquals(cur_heading.getRadians(), headings.get(1).getRadians(), kTestEpsilon);
  }
}
