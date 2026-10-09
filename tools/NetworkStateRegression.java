import com.g9third.pmweatheriv.network.AircraftStateNetwork.StatePayload;
import io.netty.buffer.Unpooled;
import java.lang.reflect.Method;
import java.util.UUID;
import net.minecraft.core.RegistryAccess;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.resources.ResourceLocation;

/** Exercises the real packet codec with independently authored airspeed and IV speed. */
public final class NetworkStateRegression {
    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    private static StatePayload sample(double airspeed, double axialVelocity) {
        return new StatePayload(new UUID(1L, 2L), ResourceLocation.fromNamespaceAndPath("minecraft", "overworld"),
            123L, true, 0.0, 0.0, -8.0, 0.1, -0.2, 0.3,
            airspeed, axialVelocity, 1.225, -4.5, false,
            45.0, true, 0.5, 12.0, 2.0, 40.0, 3.0);
    }

    public static void main(String[] args) throws Exception {
        Method finite = StatePayload.class.getDeclaredMethod("finite");
        finite.setAccessible(true);
        int checks = 0;
        // Stationary car in strong wind, moving car with tailwind, aircraft, reverse motion.
        for (double[] scenario : new double[][] {{45.0, 0.0}, {2.0, 0.8}, {60.0, 3.0}, {12.0, -0.4}}) {
            StatePayload original = sample(scenario[0], scenario[1]);
            RegistryFriendlyByteBuf buffer = new RegistryFriendlyByteBuf(Unpooled.buffer(), RegistryAccess.EMPTY);
            try {
                StatePayload.CODEC.encode(buffer, original);
                StatePayload decoded = StatePayload.CODEC.decode(buffer);
                require(decoded.equals(original), "physical state codec changed a field");
                require(buffer.readableBytes() == 0, "physical state codec left unread bytes");
                require(decoded.airspeed() == scenario[0], "wind-relative airspeed changed");
                require(decoded.axialVelocity() == scenario[1], "authoritative IV axial speed changed");
                require((boolean) finite.invoke(decoded), "valid physical state rejected");
                checks += 5;
            } finally {
                buffer.release();
            }
        }
        require(!(boolean) finite.invoke(sample(45.0, Double.NaN)), "NaN axial speed accepted");
        require(!(boolean) finite.invoke(sample(Double.POSITIVE_INFINITY, 0.0)), "infinite airspeed accepted");
        System.out.println("NetworkStateRegression: " + (checks + 2) + " assertions passed");
    }
}
