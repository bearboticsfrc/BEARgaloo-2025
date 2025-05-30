package bearlib.motor.deserializer;

import bearlib.motor.MotorConfigurator;
import bearlib.motor.deserializer.models.encoder.Encoder;
import bearlib.motor.deserializer.models.motor.Motor;
import bearlib.motor.deserializer.models.pidf.Pidf;
import com.fasterxml.jackson.databind.MapperFeature;
import com.fasterxml.jackson.databind.json.JsonMapper;
import com.fasterxml.jackson.datatype.jdk8.Jdk8Module;
import com.revrobotics.spark.SparkBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;

/**
 * MotorParser is a utility class responsible for parsing JSON files containing configuration data
 * for motors, encoders, and PIDF controllers.
 */
public class MotorParser {
  private final File directory;
  private final JsonMapper jsonMapper;

  private Motor motor;
  public Encoder encoder;
  public Pidf[] pidfs = new Pidf[4];

  /**
   * Constructs a MotorParser instance with a given directory.
   *
   * @param directory the base directory where configuration files are located.
   * @throws IllegalArgumentException if the provided directory is null or not a valid directory.
   */
  public MotorParser(File directory) {
    if (directory == null || !directory.isDirectory()) {
      throw new IllegalArgumentException("Provided directory is invalid: " + directory);
    }

    this.directory = directory;
    this.jsonMapper =
        JsonMapper.builder()
            .enable(MapperFeature.ACCEPT_CASE_INSENSITIVE_ENUMS)
            .enable(MapperFeature.ACCEPT_CASE_INSENSITIVE_PROPERTIES)
            .addModule(new Jdk8Module())
            .build();
  }

  /**
   * Retrieves a file object for the specified file path relative to the directory.
   *
   * @param filePath the relative path to the file.
   * @return a {@link File} object representing the specified file.
   */
  private File getFile(String filePath) {
    return new File(directory, filePath);
  }

  /**
   * Loads and parses a motor configuration file into a {@link Motor} object.
   *
   * @param filePath the relative path to the motor configuration file.
   * @return This {@link MotorParser} instance for method chaining.
   * @throws IOException if an error occurs while reading or parsing the file.
   */
  public MotorParser withMotor(String filePath) throws IOException {
    this.motor = jsonMapper.readValue(getFile(filePath), Motor.class);
    return this;
  }

  /**
   * Loads and parses an encoder configuration file into an {@link Encoder} object.
   *
   * @param filePath the relative path to the encoder configuration file.
   * @return This {@link MotorParser} instance for method chaining.
   * @throws IOException if an error occurs while reading or parsing the file.
   */
  public MotorParser withEncoder(String filePath) throws IOException {
    this.encoder = jsonMapper.readValue(getFile(filePath), Encoder.class);
    return this;
  }

  /**
   * Loads and parses a PIDF configuration file into a {@link Pidf} object and assigns it to a
   * specified PIDF slot.
   *
   * @param filePath the relative path to the PIDF configuration file.
   * @param slot the slot to store the PIDF object.
   * @return This {@link MotorParser} instance for method chaining.
   * @throws IOException if an error occurs while reading or parsing the file.
   * @throws ArrayIndexOutOfBoundsException if the slot index is out of bounds.
   */
  public MotorParser withPidf(String filePath, int slot) throws IOException {
    if (slot < 0 || slot >= pidfs.length) {
      throw new ArrayIndexOutOfBoundsException("Invalid PIDF slot: " + slot);
    }

    this.pidfs[slot] = jsonMapper.readValue(getFile(filePath), Pidf.class);

    System.out.println("PIDF: p " + this.pidfs[slot].getP().get());
    return this;
  }

  /**
   * Loads and parses a PIDF configuration file into the default PIDF slot (index 0).
   *
   * @param filePath the relative path to the PIDF configuration file.
   * @return This {@link MotorParser} instance for method chaining.
   * @throws IOException if an error occurs while reading or parsing the file.
   */
  public MotorParser withPidf(String filePath) throws IOException {
    withPidf(filePath, 0);
    return this;
  }

  /**
   * Configures the motor and associated components using the parsed data.
   *
   * @return a fully configured {@link SparkBase}.
   */
  public SparkBase configureAsync() {
    if (motor == null) {
      throw new IllegalStateException("Motor was not set before attempting to configure.");
    }

    MotorConfigurator motorConfigurator = new MotorConfigurator().withMotor(motor);

    if (Optional.ofNullable(encoder).isPresent()) {
      motorConfigurator.withEncoder(encoder);
    }

    for (int slot = 0; slot < pidfs.length; slot++) {
      Pidf pidf = pidfs[slot];

      if (pidf == null) {
        continue;
      }

      motorConfigurator.withPidf(pidfs[slot], slot);
    }

    return motorConfigurator.configureAsync();
  }
}
