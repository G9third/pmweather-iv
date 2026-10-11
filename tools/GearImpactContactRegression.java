import com.g9third.pmweatheriv.physics.GearImpactContact;
import com.g9third.pmweatheriv.physics.SpawnTerrainDamageGrace;
import com.g9third.pmweatheriv.physics.TireNormalCompliance;
import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactRuntimeConfig;

/** Narrow gear strikes, dissipative material targets and surface fracture eligibility. */
public final class GearImpactContactRegression {
    private static int assertions;
    private static void require(boolean v,String message) {
        ++assertions; if(!v) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        require(!SpawnTerrainDamageGrace.allowsTerrainDamage(true,0),"new placement suppresses world mutation");
        require(!SpawnTerrainDamageGrace.allowsTerrainDamage(true,59),"settling grace spans three seconds");
        require(SpawnTerrainDamageGrace.allowsTerrainDamage(true,60),"block breaking resumes at the boundary");
        require(SpawnTerrainDamageGrace.allowsTerrainDamage(false,0),"saved loaded vehicles do not gain spawn immunity");
        require(SpawnTerrainDamageGrace.allowsTerrainDamage(true,1000),"old vehicle rebuild cannot restart grace");
        double width=.25, diameter=.5;
        double first=GearImpactContact.area(width,diameter,.02,0,true);
        require(first>0 && first<width*diameter*.5,"first wheel strike uses a narrow loaded cap");
        require(GearImpactContact.area(width,diameter,.02,.08,true)>first,
            "footprint grows with actual penetration");
        require(GearImpactContact.area(width,diameter,.02,2,true)==width*diameter,
            "deeply buried wheel reaches its full diameter without exceeding it");
        require(GearImpactContact.area(width,diameter,.02,0,false)==width*diameter,
            "treads retain their authored rectangular strike area");
        require(GearImpactContact.area(Double.NaN,diameter,.02,0,true)==0,
            "malformed dimensions cannot introduce an impact area");
        require(GearImpactContact.normalTarget(0,true,.4,12,.025)==0,
            "fully arrested material impact cannot become an upward penetration bias");
        require(GearImpactContact.normalTarget(-5,true,.4,12,.025)==-5,
            "opened terrain keeps its inward residual");
        require(GearImpactContact.normalTarget(2,true,.4,12,.025)==0,
            "material response cannot prescribe rebound");
        require(GearImpactContact.normalTarget(0,false,.02,12,.025)>0,
            "ordinary support keeps its resting penetration stabilization");
        require(GearImpactContact.normalTarget(-.5,false,.02,12,.025)==-.5,
            "predictive touchdown keeps its approach target");
        double target=GearImpactContact.normalTarget(0,true,.4,12,.025);
        require(TireNormalCompliance.response(1000,.02,-.4,target,.01,.025,true,true).targetMps()==0,
            "material-arrest target reaches the actual tire response unchanged");
        require(ExternalWorldImpactModel.surfaceFractureEligible(3000,.075),
            "energetic shallow tail contact can fracture before rigid impulse");
        require(!ExternalWorldImpactModel.surfaceFractureEligible(40,.075),
            "routine support cannot use immediate energetic fracture");
        require(!ExternalWorldImpactModel.surfaceFractureEligible(3000,0),
            "no available physical travel means no immediate fracture");
        boolean old=ImpactRuntimeConfig.ENABLE_BLOCK_BREAKING;
        try {
            ImpactRuntimeConfig.ENABLE_BLOCK_BREAKING=false;
            require(!ExternalWorldImpactModel.surfaceFractureEligible(3000,.075),
                "disabled block breaking still keeps material rigid");
        } finally { ImpactRuntimeConfig.ENABLE_BLOCK_BREAKING=old; }
        require(ImpactRuntimeConfig.PENETRATION_MIN_SPEED_MS==8,
            "deep penetration minimum speed remains upstream default");
        System.out.println("GearImpactContactRegression: "+assertions+" assertions passed");
    }
}
