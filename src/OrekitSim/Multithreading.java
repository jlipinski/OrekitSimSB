package OrekitSim;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.estimation.measurements.AngularAzEl;
import org.orekit.estimation.measurements.Position;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.measurements.generation.AngularAzElBuilder;
import org.orekit.estimation.measurements.generation.PositionBuilder;
import org.orekit.estimation.measurements.generation.EventBasedScheduler;
import org.orekit.estimation.measurements.generation.Generator;
import org.orekit.estimation.measurements.generation.SignSemantic;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.geometry.fov.DoubleDihedraFieldOfView;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.BooleanDetector;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.FieldOfViewDetector;
import org.orekit.propagation.events.InterSatDirectViewDetector;
import org.orekit.propagation.events.handlers.ContinueOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.FixedStepSelector;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.SortedSet;

public class Multithreading{

    public static double[] Sph2Car(double phi, double theta, double r){
        double radPhi = FastMath.toRadians(90.-phi);
        double radTheta = FastMath.toRadians(theta);
        return new double[]{
                r * FastMath.cos(radPhi) * FastMath.cos(radTheta),
                r * FastMath.sin(radPhi) * FastMath.cos(radTheta),
                r * FastMath.sin(radTheta)
        };
    }

    public static void writeFile(String fileName, String line)
    {
        try
        {
            FileWriter fw = new FileWriter(fileName, true);
            fw.write(line);
            fw.close();
        }
        catch (IOException e)
        {
            System.out.println("Ooops... Wystąpił błąd: ");
            e.printStackTrace();
        }
    }

    List <TLE> TLEs;
    Frame ITRF = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
    OneAxisEllipsoid earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
            Constants.WGS84_EARTH_FLATTENING,
            ITRF);
    Frame EME = FramesFactory.getEME2000();
    CelestialBody sun = CelestialBodyFactory.getSun();

    BooleanDetector detector;
    String plik;
    TLE PmTLE;

    AbsoluteDate t0, t1;


    public Multithreading(String plik, String PathToTle, TLE PmTLE, AbsoluteDate t0, AbsoluteDate t1) {



        File orekitData = new File("src/orekit-data");
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        this.plik = plik;
        this.TLEs = new ArrayList<>();
        this.PmTLE = PmTLE;
        this.t0 = t0;
        this.t1 = t1;

        try {
            File tle_file = new File(PathToTle);
            Scanner scn = new Scanner(tle_file);
            String line1 = "";
            String line2;
            int i = 0;
            while (scn.hasNextLine()) {
                if (i == 1) {
                    line1 = scn.nextLine();
                } else if (i == 2) {
                    line2 = scn.nextLine();
                    TLEs.add(new TLE(line1, line2));
                } else {
                    i = 0;
                    scn.nextLine();
                    line1 = "";
                    //                    line2 = "";
                }
                i++;
            }
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

    }

    public FieldOfViewDetector addDetector(
            TLEPropagator propagator, double theta, double phi, double width, double height
    )
    {
        double[] coords = Sph2Car(phi, theta, 1.);
        double x = coords[0];
        double y = coords[1];
        double z = coords[2];

        Vector3D center = new Vector3D(x, y, z);
        Vector3D axis2 = Vector3D.crossProduct(Vector3D.PLUS_K, center).normalize();
        Vector3D axis1 = Vector3D.crossProduct(axis2, center).normalize();

        double ha1 = FastMath.toRadians(width/2.);
        double ha2 = FastMath.toRadians(height/2.);

        DoubleDihedraFieldOfView fov = new DoubleDihedraFieldOfView(center, axis1, ha1, axis2, ha2, 0.0);

        FieldOfViewDetector fovd = new FieldOfViewDetector(
                propagator, fov
        ).withHandler(new ContinueOnEvent<>()).withMaxCheck(15.0);
        return fovd;
    }
    public void run()
    {
        long start = System.currentTimeMillis();
        try {
            System.out.println("Thread " + Thread.currentThread().getId() + " is running");
            writeFile(this.plik, "ID,date,X,Y,Z\r\n");
            for (TLE myTle : this.TLEs) {
                TLEPropagator PmPropagator = TLEPropagator.selectExtrapolator(PmTLE);
                TLEPropagator propagator = TLEPropagator.selectExtrapolator(myTle);
                FieldOfViewDetector fovd = addDetector(PmPropagator, 0., 0., 5., 5.);
                InterSatDirectViewDetector isdvd = new InterSatDirectViewDetector(earth, propagator);
//                this.detector = BooleanDetector.andCombine(fovd, isdvd);

                final Generator generator = new Generator();
                ObservableSatellite sat = generator.addPropagator(PmPropagator);

                PositionBuilder PosBuilder = new PositionBuilder(
                        null, 0., 1., sat
                );

                EventBasedScheduler<Position> scheduler = new EventBasedScheduler<>(
                        PosBuilder, new FixedStepSelector(60.0, TimeScalesFactory.getUTC()),
                        PmPropagator, this.detector,
                        SignSemantic.FEASIBLE_MEASUREMENT_WHEN_POSITIVE
                );

                generator.addScheduler(scheduler);


                try {
                    SortedSet<ObservedMeasurement<?>> data = generator.generate(t0, t1);
//                    for (final ObservedMeasurement<?> meas : data) {
//
//                        final Position pos = (Position) meas;
//
//                        Vector3D v = pos.getPosition();
//                        final AbsoluteDate date = pos.getDate();
//                        Vector3D new_v = EME.getTransformTo(ITRF, date).transformPosition(v);
//                        double x = new_v.getX()/1000.0;
//                        double y = new_v.getY()/1000.0;
//                        double z = new_v.getZ()/1000.0;
//
//                        String tmp = myTle.getSatelliteNumber() + "," + date + "," + x + "," + y + "," + z + "\r\n";
//                        writeFile(plik, tmp);
//                    }
                } catch (OrekitException exception) {
                    System.out.println(myTle.getSatelliteNumber() + " | " +exception);
                }
            }

        }catch (Exception e) {
            System.out.println("Exception is caught");
        }
        long stop = System.currentTimeMillis();
        System.out.println("Thread " + Thread.currentThread().getId() + " finished after "+(stop-start)/1000.0 + "s");
    }
}
