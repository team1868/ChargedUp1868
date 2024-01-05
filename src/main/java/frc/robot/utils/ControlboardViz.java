package frc.robot.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.GamePieceType;
import frc.robot.constants.enums.HPIntakeStations;
import frc.robot.constants.enums.ScoringLevels;
import frc.robot.constants.enums.ScoringLocations;

public class ControlboardViz {
  private Alliance _alliance;
  private Field2d _field;
  private boolean _vizText;
  private boolean _vizGlass;
  private boolean _vizField;
  // frc::Mechanism2d _mech2d{CBVIZ_XSIZE, CBVIZ_YSIZE};
  // frc::MechanismLigament2d* _scoreLigament;
  // frc::MechanismLigament2d* _heldPieceLigament;
  private GenericEntry _desiredScoringLevelEntry;
  private GenericEntry _desiredScoringLocationEntry;
  private GenericEntry _desiredIntakeEntry;
  private GenericEntry _desiredGamePieceEntry;
  private GenericEntry _heldGamePieceEntry;
  // ConeCube _scorePieceShape, _heldPieceShape, _desiredPieceShape;
  // ConeCube _allianceOutline;

  public ControlboardViz(Field2d field, boolean vizNumbers, boolean vizGlass, boolean vizField) {
    _field = field;
    _vizText = vizNumbers;
    _vizGlass = vizGlass;
    _vizField = vizField;

    initVizText();
    initGlass();

    LoopTimer.markEvent(" Visualizer Initialization Complete: ");
  }

  public void update(
      ScoringLocations desiredLocation,
      ScoringLevels desiredLevel,
      HPIntakeStations desiredIntake,
      GamePieceType desiredGamePiece,
      GamePieceType heldGamePiece
  ) {
    LoopTimer.markLoopStart();

    final int loc = desiredLocation.ordinal();
    if (_vizText) {
      if (desiredLocation == ScoringLocations.UNKNOWN_SCORING_LOCATION)
        _desiredScoringLocationEntry.setString("UNKNOWN SCORING LOCATION");
      else
        _desiredScoringLocationEntry.setString(
            "LOCATION " + Integer.toString(loc) + ((loc % 3 == 2) ? "CUBE" : "CONE")
        );
      _desiredScoringLevelEntry.setString(desiredLevel.label);
      _desiredGamePieceEntry.setString(desiredGamePiece.label + " DESIRED");
      _heldGamePieceEntry.setString(heldGamePiece.label + " HELD");
      _desiredIntakeEntry.setString(desiredIntake.toString());
    }
    // UpdateGlass(desiredLocation, desiredLevel, desiredIntake, desiredGamePiece,
    // heldGamePiece);

    LoopTimer.markEvent(" Total Visualizer: ");
  }

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
    // frc::Color8Bit color;
    // switch (_alliance) {
    // case frc::DriverStation::kRed: color = frc::Color8Bit(frc::Color::kFirstRed);
    // break;
    // case frc::DriverStation::kBlue: color =
    // frc::Color8Bit(frc::Color::kFirstBlue); break;
    // default: color = frc::Color8Bit(frc::Color::kYellow); break;
    // }
    // ColorShape(_allianceOutline, color);
  }

  // double ControlboardViz::scaleX(meter_t x) { return CBVIZ_XSIZE * (x + 1_ft) /
  // 18_ft; }
  // double ControlboardViz::scaleY(meter_t y) { return CBVIZ_YSIZE * (y + 1_ft) /
  // 8_ft; }

  // frc::MechanismLigament2d* ControlboardViz::Draw(
  // meter_t x0, meter_t y0, meter_t x1, meter_t y1, int width, frc::Color8Bit
  // color)
  // {

  // // static int _id;

  // // _id++; // each segment needs a unique id

  // // double const windowX0 = scaleX(x0);
  // // double const windowX1 = scaleX(x1);
  // // double const windowY0 = scaleY(y0);
  // // double const windowY1 = scaleY(y1);
  // // double const deltaX = windowX1 - windowX0;
  // // double const deltaY = windowY1 - windowY0;

  // // frc::MechanismRoot2d* start = _mech2d.GetRoot("Line" +
  // std::to_string(_id), windowX0,
  // // windowY0); auto angle = radian_t{std::atan2(deltaY, deltaX)}; int
  // length
  // =
  // // std::sqrt(deltaX * deltaX + deltaY * deltaY);

  // // return start->Append<frc::MechanismLigament2d>("Line", length, angle,
  // width, color);
  // return {};
  // }

  private void initVizText() {
    if (_vizText) {
      ShuffleboardTab tab = Shuffleboard.getTab("Competition HUD");

      _desiredScoringLevelEntry =
          tab.add("Scoring Level", " ").withSize(2, 1).withPosition(2, 0).getEntry();
      _desiredScoringLocationEntry =
          tab.add("Scoring Location", " ").withSize(2, 1).withPosition(0, 0).getEntry();
      _desiredGamePieceEntry =
          tab.add("Desired Game Piece", " ").withSize(2, 1).withPosition(6, 0).getEntry();
      _heldGamePieceEntry =
          tab.add("Held Game Piece", " ").withSize(2, 1).withPosition(4, 0).getEntry();
      _desiredIntakeEntry =
          tab.add("Intake Location", " ").withSize(2, 1).withPosition(8, 0).getEntry();
    }
  }

  private void initGlass() {
    // if (!_vizGlass) return;

    // grid verticals
    // Draw(0_ft, 0_ft, 0_ft, height); // left edge
    // for (int i = 0; i < 8; i++) {
    // meter_t x = GRID_VIZ_PITCH / 6 + (i + 1) * GRID_VIZ_PITCH;
    // Draw(x, 0_ft, x, height, (i % 3 == 2) ? 2 : 1);
    // }
    // Draw(width, 0_ft, width, height);

    // grid horizontals
    // Draw(0_ft, height / 3, width, height / 3);
    // Draw(0_ft, 2 * height / 3, width, 2 * height / 3);
    // Draw(0_ft, height, width, height);

    // frc::MechanismRoot2d* markerStart = _mech2d.GetRoot("Marker", scaleX(baseX),
    // scaleY(baseY));
    // _scoreLigament = markerStart->Append<frc::MechanismLigament2d>(
    // "ScoreLigament", 0, 90_deg, 0, frc::Color8Bit(frc::Color::kBlack));
    // InitRectangle(_scorePieceShape, _scoreLigament);

    // outer border, color-coded for alliance
    // _allianceOutline.rt = Draw(0_ft, 0_ft, width, 0_ft, 6);
    // _allianceOutline.up = Draw(width, 0_ft, width, height, 6);
    // _allianceOutline.lf = Draw(width, height, 0_ft, height, 6);
    // _allianceOutline.dn = Draw(0_ft, height, 0_ft, 0_ft, 6);
    // UpdateAlliance(_alliance); // sets proper color

    // held game piece
    // frc::MechanismRoot2d* heldPieceStart =
    // _mech2d.GetRoot("HeldPiece", CBVIZ_XSIZE - cubeSidePix * 2, scaleY(baseY));
    // _heldPieceLigament = heldPieceStart->Append<frc::MechanismLigament2d>(
    // "HeldPieceLigament", CBVIZ_YSIZE / 2, 270_deg, 0,
    // frc::Color8Bit(frc::Color::kBlack));
    // InitRectangle(_heldPieceShape, _heldPieceLigament);

    // frc::MechanismLigament2d* clawSupport =
    // _heldPieceLigament->Append<frc::MechanismLigament2d>(
    // "ClawSupport", clawSupportLen, CLAW_VIZ_ANGLE, 0,
    // frc::Color8Bit(frc::Color::kBlack));
    // clawSupport->Append<frc::MechanismLigament2d>("ClawLeft",
    // clawLenPix,
    // 190_deg - 2 * CLAW_VIZ_ANGLE,
    // 1,
    // frc::Color8Bit(frc::Color::kWhite));
    // clawSupport->Append<frc::MechanismLigament2d>(
    // "ClawRight", clawLenPix, 170_deg, 1, frc::Color8Bit(frc::Color::kWhite));

    // // desired game piece
    // frc::MechanismRoot2d* desiredPieceStart =
    // _mech2d.GetRoot("DesiredPiece", CBVIZ_XSIZE - cubeSidePix * 2,
    // scaleY(baseY));
    // frc::MechanismLigament2d* desiredPieceLigament =
    // desiredPieceStart->Append<frc::MechanismLigament2d>(
    // "DesiredPieceLigament", 0, 270_deg, 0, frc::Color8Bit(frc::Color::kBlack));
    // InitRectangle(_desiredPieceShape, desiredPieceLigament);
    // RotateShape(_desiredPieceShape, 90_deg);

    // display it!
    // frc::SmartDashboard::PutData("Controlboard", &_mech2d);
  }

  private void updateGlass(
      ScoringLocations desiredLocation,
      ScoringLevels desiredLevel,
      HPIntakeStations desiredIntake,
      GamePieceType desiredGamePiece,
      GamePieceType heldGamePiece
  ) {
    // if (!_vizGlass) return;

    // 1. update desired location & level on grid

    // left to right: red is 1-9, blue is 9-1
    // int locationIndex = (desiredLocation == UNKNOWN_SCORING_LOCATION) ? 0
    // : (_alliance == frc::DriverStation::kRed) ? desiredLocation
    // : 10 - desiredLocation;

    // meter_t x, y; // location in physical world

    // Spacing of the viz grid is like this:
    // Middle sections are spaced at GRID_VIZ_PITCH (call that P)
    // First and last sections are spaced at 7P/6, ie 16% larger
    // switch (locationIndex) {
    // // should never happen
    // case 0: x = -GRID_VIZ_MARGIN; break;
    // // first section goes from 0 to 7P/6, so its midpoint is 7P/12
    // case 1: x = GRID_VIZ_PITCH * 7.0 / 12.0; break;
    // // last section:
    // // first offset 7P/6 to get past first section: +14P/12
    // // then +P for each section beyone section 2 : +P(location-2)
    // // finally +7P/12 to midpoint of last section : +7P/12
    // case 9: x = GRID_VIZ_PITCH * ((21.0 / 12.0) - 2.0 + locationIndex); break;
    // // middle sections:
    // // first offset 7P/6 to get past first section: +7P/6
    // // then +P for each section beyond section 2 : +P(location-2)
    // // finally +0.5P to midpoint of section : +P/2
    // default: x = GRID_VIZ_PITCH * ((7.0 / 6.0) - 1.5 + locationIndex);
    // }

    // y = (desiredLevel == UNKNOWN_SCORING_LEVEL) ? baseY
    // : GRID_VIZ_MARGIN + (desiredLevel - 1) * height /
    // 3;

    // PlaceShape(
    // _scorePieceShape, scaleX(x) - scaleX(baseX) - cubeSidePix / 2, scaleY(y) -
    // scaleY(baseY));

    // frc::Color8Bit shapeColor = frc::Color8Bit(frc::Color::kYellow);

    // if (desiredLevel == SCORING_LEVEL_0) {
    // MakeHybrid(_scorePieceShape);
    // shapeColor = frc::Color8Bit(frc::Color::kSpringGreen); // good location
    // } else if (locationIndex % 3 == 2) {
    // MakeCube(_scorePieceShape);
    // if (heldGamePiece == CUBE_GAME_PIECE)
    // shapeColor = frc::Color8Bit(frc::Color::kSpringGreen);
    // else if (heldGamePiece == CONE_GAME_PIECE)
    // shapeColor = frc::Color8Bit(frc::Color::kRed);
    // } else {
    // MakeCone(_scorePieceShape);
    // if (heldGamePiece == CONE_GAME_PIECE)
    // shapeColor = frc::Color8Bit(frc::Color::kSpringGreen);
    // else if (heldGamePiece == CUBE_GAME_PIECE)
    // shapeColor = frc::Color8Bit(frc::Color::kRed);
    // }
    // ColorShape(_scorePieceShape, shapeColor);

    // // 2. update held game piece
    // PlaceShape(_heldPieceShape, 0, -CBVIZ_YSIZE / 2);
    // if (heldGamePiece == CONE_GAME_PIECE) {
    // MakeCone(_heldPieceShape);
    // } else if (heldGamePiece == CUBE_GAME_PIECE) {
    // MakeCube(_heldPieceShape);
    // }
    // frc::Color8Bit heldColor =
    // (heldGamePiece == CONE_GAME_PIECE || heldGamePiece == CUBE_GAME_PIECE)
    // ? frc::Color8Bit(frc::Color::kLightBlue)
    // : frc::Color8Bit(frc::Color::kBlack);
    // ColorShape(_heldPieceShape, heldColor);

    // // 3. update desired game piece
    // if (desiredGamePiece == CONE_GAME_PIECE) {
    // MakeCone(_desiredPieceShape);
    // } else if (desiredGamePiece == CUBE_GAME_PIECE) {
    // MakeCube(_desiredPieceShape);
    // }
    // ColorShape(_desiredPieceShape, frc::Color8Bit(frc::Color::kLightBlue));
  }
}
// void ControlboardViz::InitRectangle(ConeCube& shape,
// frc::MechanismLigament2d* prevLigament)
// {
// // shape.parent = prevLigament;
// // shape.rt = prevLigament->Append<frc::MechanismLigament2d>("ShapeRt",
// cubeSidePix,
// 0_deg,
// // 2); shape.up = shape.rt->Append<frc::MechanismLigament2d>("ShapeUp",
// cubeSidePix,
// 90_deg,
// // 2); shape.lf = shape.up->Append<frc::MechanismLigament2d>("ShapeLf",
// cubeSidePix,
// 90_deg,
// // 2); shape.dn = shape.lf->Append<frc::MechanismLigament2d>("ShapeDn",
// cubeSidePix,
// 90_deg,
// // 2);
// }

// void ControlboardViz::MakeCube(ControlboardViz::ConeCube& shape)
// {
// // shape.rt->SetLength(cubeSidePix);
// // shape.up->SetLength(cubeSidePix);
// // shape.lf->SetLength(cubeSidePix);
// // shape.dn->SetLength(cubeSidePix);
// // // don't need to set angle for shape.rt - it is set by rotate function
// // shape.up->SetAngle(90_deg);
// // shape.lf->SetAngle(90_deg);
// // shape.dn->SetAngle(90_deg);
// }

// void ControlboardViz::MakeCone(ControlboardViz::ConeCube& shape)
// {
// // shape.rt->SetLength(coneBasePix);
// // shape.up->SetLength(coneSlantPix);
// // shape.lf->SetLength(0);
// // shape.dn->SetLength(coneSlantPix);
// // // don't need to set angle for shape.rt - it is set by rotate function
// // shape.up->SetAngle(180_deg - coneAngle);
// // shape.lf->SetAngle(0_deg);
// // shape.dn->SetAngle(2 * coneAngle);
// }

// void ControlboardViz::MakeHybrid(ControlboardViz::ConeCube& shape)
// {
// // shape.rt->SetLength(cubeSidePix);
// // shape.up->SetLength(coneSlantPix);
// // shape.lf->SetLength(cubeSidePix / 2);
// // shape.dn->SetLength(coneHeightPix);
// // // don't need to set angle for shape.rt - it is set by rotate function
// // shape.up->SetAngle(180_deg - coneAngle);
// // shape.lf->SetAngle(coneAngle);
// // shape.dn->SetAngle(90_deg);
// }

// void ControlboardViz::MakeUnknown(ControlboardViz::ConeCube& shape)
// {
// // shape.rt->SetLength(cubeSidePix / 2);
// // shape.up->SetLength(cubeSidePix / 2);
// // shape.lf->SetLength(cubeSidePix / 2);
// // shape.dn->SetLength(cubeSidePix / 2);
// // // don't need to set angle for shape.rt - it is set by rotate function
// // shape.up->SetAngle(90_deg);
// // shape.lf->SetAngle(-45_deg);
// // shape.dn->SetAngle(90_deg);
// }

// void ControlboardViz::ColorShape(ControlboardViz::ConeCube& shape,
// frc::Color8Bit color)
// {
// // shape.rt->SetColor(color);
// // shape.up->SetColor(color);
// // shape.lf->SetColor(color);
// // shape.dn->SetColor(color);
// }

// void ControlboardViz::RotateShape(ControlboardViz::ConeCube& shape, degree_t
// angle)
// {
// // shape.rt->SetAngle(angle);
// }

// void ControlboardViz::PlaceShape(ConeCube& shape, double dxPix, double dyPix)
// {
// // double dPix = std::sqrt(dxPix * dxPix + dyPix * dyPix);
// // radian_t angle = radian_t{std::atan2(dyPix, dxPix)};

// // shape.parent->SetAngle(angle);
// // shape.parent->SetLength(dPix);
// // RotateShape(shape, -angle);
// }
